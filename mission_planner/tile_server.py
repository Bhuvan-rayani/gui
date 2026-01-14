import sqlite3
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path

# Absolute, correct path to the MBTiles file
MBTILES = Path(__file__).resolve().parent / "data" / "NIDAR.mbtiles"
print("USING MBTILES:", MBTILES)


class TileHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        parts = self.path.strip("/").split("/")
        if len(parts) != 3:
            self.send_error(404)
            return

        z, x, y = map(int, parts)

        # ⚠️ COMMENT THIS OUT if tiles don't appear
        # y = (1 << z) - 1 - y

        if not MBTILES.exists():
            raise FileNotFoundError(f"MBTiles not found: {MBTILES}")

        conn = sqlite3.connect(str(MBTILES))
        cur = conn.cursor()

        cur.execute(
            """
            SELECT tile_data
            FROM tiles
            WHERE zoom_level=? AND tile_column=? AND tile_row=?
            """,
            (z, x, y),
        )
        row = cur.fetchone()
        conn.close()

        if not row:
            self.send_error(404)
            return

        self.send_response(200)
        self.send_header("Content-Type", "image/png")
        self.end_headers()
        self.wfile.write(row[0])


def run():
    HTTPServer(("127.0.0.1", 8000), TileHandler).serve_forever()


if __name__ == "__main__":
    run()
