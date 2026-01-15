#!/usr/bin/env python3

import os
import math
import json
from glob import glob
import time
from std_msgs.msg import Int32

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from pyproj import Transformer, CRS, Geod

from math import radians, degrees, sin, cos, asin, atan2

dfov = 95.0
w = 640
h = 480
pixel_um = 2.8

sensor_w_mm = (float(pixel_um) / 1000.0) * float(w)
sensor_h_mm = (float(pixel_um) / 1000.0) * float(h)
sensor_diag_mm = math.hypot(sensor_w_mm, sensor_h_mm)

f = sensor_diag_mm / (2.0 * math.tan(math.radians(dfov) / 2))

image_dir = os.path.join(os.getcwd(), "images")
metadata_dir = os.path.join(os.getcwd(), "metadata")

min_alt = 8.0
min_contour = 100
R_EARTH = 6377830.0

_transformer_cache = {}

def utm_to_gps(lat, lon):
    zone = int((lon + 180) / 6) + 1
    epsg = 32600 + zone if lat >= 0 else 32700 + zone

    if epsg not in _transformer_cache:
        proj_to_utm = Transformer.from_crs("EPSG:4326", f"EPSG:{epsg}", always_xy=True)
        proj_from_utm = Transformer.from_crs(f"EPSG:{epsg}", "EPSG:4326", always_xy=True)
        _transformer_cache[epsg] = (proj_to_utm, proj_from_utm)

    return _transformer_cache[epsg]

def calculate_gsd(altitude_m, sensor_dim_mm, focal_length_mm, image_dim_px):

    sensor_dim_m = float(sensor_dim_mm) / 1000.0
    focal_length_m = float(focal_length_mm) / 1000.0

    gsd_m = (altitude_m * sensor_dim_m) / (focal_length_m * float(image_dim_px))
    return gsd_m


def pixel_to_utm_to_gps(u, v, drone_lat, drone_lon, alt_m, roll_rad, pitch_rad, yaw_rad, img_w, img_h, sensor_w_mm, sensor_h_mm, focal_mm):
    
    gsd_x = calculate_gsd(alt_m, sensor_w_mm, focal_mm, img_w)
    gsd_y = calculate_gsd(alt_m, sensor_h_mm, focal_mm, img_h)

    cx = img_w / 2.0
    cy = img_h / 2.0
    dx_px = float(u) - cx
    dy_px = float(v) - cy

    x_east_m = dx_px * gsd_x
    y_down_m = dy_px * gsd_y

    north_m = -y_down_m
    east_m = x_east_m

    cos_y = math.cos(yaw_rad - 0.0872664626)
    sin_y = math.sin(yaw_rad - 0.0872664626)
    #cos_y = math.cos(yaw_rad)
    #sin_y = math.sin(yaw_rad)

    east_from_pixels = east_m * cos_y - north_m * sin_y
    north_from_pixels = east_m * sin_y + north_m * cos_y

    #east_tilt = alt_m * math.tan(roll_rad)
    #north_tilt = alt_m * math.tan(pitch_rad)
    east_tilt = alt_m * (math.tan(-roll_rad + 0.05235987756))
    north_tilt = alt_m * (math.tan(-pitch_rad - 0.0872664626))
    
    east_from_tilt = east_tilt * cos_y - north_tilt * sin_y
    north_from_tilt = east_tilt * sin_y + north_tilt * cos_y

    east_total = east_from_pixels + east_from_tilt
    north_total = north_from_pixels + north_from_tilt

    proj_to_utm, proj_from_utm = utm_to_gps(drone_lat, drone_lon)
    x0, y0 = proj_to_utm.transform(drone_lon, drone_lat)

    x_t = x0 + east_total
    y_t = y0 + north_total

    lon_t, lat_t = proj_from_utm.transform(x_t, y_t)

    return lat_t, lon_t


def detect_yellow(img_bgr, edge_margin_px = 100):

    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

    # yellow
    lower = np.array([20, 170, 40])
    upper = np.array([40, 255, 255])
    
    mask = cv2.inRange(hsv, lower, upper)

    kernel = np.ones((6, 6), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.dilate(mask, kernel, iterations=1)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    img_h, img_w = img_bgr.shape[:2]
    
    centroids = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_contour:
            continue
        M = cv2.moments(cnt)
        if M.get("m00", 0) == 0:
            continue
        cx = M["m10"] / M["m00"]
        cy = M["m01"] / M["m00"]
        
        if (cx < edge_margin_px or cx > img_w - edge_margin_px or cy < edge_margin_px or cy > img_h - edge_margin_px):
            continue
            
        centroids.append((cx, cy))

    return centroids
    
def haversine_distance_m(lat1, lon1, lat2, lon2):

    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2.0) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2.0) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(max(0.0, 1 - a)))
    return R_EARTH * c


class WaypointPublisher(Node):

    def __init__(self, image_dir=image_dir, metadata_dir=metadata_dir, poll_hz=10.0):
        
        super().__init__('yellow_waypoint_publisher')
        self.pub = self.create_publisher(Point, "/waypoints", 10)
        self.image_dir = image_dir
        self.metadata_dir = metadata_dir
        self.processed = set() 
        self.known_waypoints = []

        period_s = 1.0 / poll_hz
        self.create_timer(period_s, self.timer_callback)


    def timer_callback(self):

        try:
            self.poll_folders()
        except Exception as e:
            self.get_logger().error(f"Error in polling loop: {e}")


    def poll_folders(self):

        exts = ['*.png']
        files = []
        for e in exts:
            files.extend(glob(os.path.join(self.image_dir, e)))
        files = sorted(files)

        for img_path in files:
            basename = os.path.splitext(os.path.basename(img_path))[0]
            if basename in self.processed:
                continue
            json_path = os.path.join(self.metadata_dir, basename + ".json")
            if not os.path.exists(json_path):
                self.get_logger().warning(f"Missing JSON for {img_path} -> expected {json_path}. Skipping.")
                self.processed.add(basename)
                continue

            try:
                self._process_one(img_path, json_path, basename)
            except Exception as e:
                self.get_logger().error(f"Failed to process {basename}: {e}")
            finally:                
                self.processed.add(basename)
                
    
    def is_unique_waypoint(self, lat, lon, min_dist_m=3.0):
    
        for wp in self.known_waypoints:
            d = haversine_distance_m(lat, lon, wp['lat'], wp['lon'])
            if d < min_dist_m:
                return False
        return True


    def _process_one(self, img_path, json_path, basename):

        try:
            with open(json_path, 'r') as jf:
                meta = json.load(jf)
        except Exception as e:
            self.get_logger().error(f"Failed to load JSON {json_path}: {e}")
            return

        gps = meta.get("gps", {})
        odom = meta.get("odom", {})

        drone_lat = float(gps["lat"])
        drone_lon = float(gps["lon"])
        alt_m = float(gps["alt"])

        pitch_rad = odom.get("pitch", 0.0)
        roll_rad  = odom.get("roll", 0.0)
        yaw_rad = odom.get("yaw", 0.0)
            
        if alt_m < min_alt:
            return

        img = cv2.imread(img_path)
        if img is None:
            self.get_logger().error(f"Failed to read image {img_path}. Skipping.")
            return

        centroids = detect_yellow(img)
        if not centroids:            
            return

        for (u, v) in centroids:
            lat_t, lon_t = pixel_to_utm_to_gps(u, v, drone_lat, drone_lon, alt_m, roll_rad, pitch_rad, yaw_rad, w, h, sensor_w_mm, sensor_h_mm, f) 
            
            if not self.is_unique_waypoint(lat_t, lon_t):
                continue           
            
            p = Point()
            
            p.x = float(lat_t)
            p.y = float(lon_t)
            
            self.pub.publish(p)
            self.known_waypoints.append({'lat': p.x, 'lon': p.y})
            self.get_logger().info(f"Published waypoint [{basename}] lat={p.x:.8f}, lon={p.y:.8f}")
            
        detections_dir = os.path.join(os.getcwd(), "detections")
        os.makedirs(detections_dir, exist_ok=True)

        debug_img = img.copy()

        for (cx, cy) in centroids:
            cx_i, cy_i = int(round(cx)), int(round(cy))

            arrow_start = (cx_i - 40, cy_i - 40)
            arrow_end = (cx_i, cy_i)

            cv2.arrowedLine(
                debug_img,
                arrow_start,
                arrow_end,
                (0, 0, 255),
                1,
                tipLength=0.25
            )

            cv2.putText(
                debug_img,
                "Diseased Plant",
                (arrow_start[0], arrow_start[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 255),
                1,
                cv2.LINE_AA
            )

        cv2.imwrite(
            os.path.join(detections_dir, f"detection_{basename}.jpg"),
            debug_img
        )


def main():
    if not os.path.isdir(image_dir):
        print(f"Images folder not found: {image_dir}")
        return
    if not os.path.isdir(metadata_dir):
        print(f"Metadata folder not found: {metadata_dir}")
        return

    rclpy.init()
    node = WaypointPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
