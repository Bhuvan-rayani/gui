import pexpect

HOST = "192.168.31.149"
USER = "manas"
PASSWORD = "cleo"

child = pexpect.spawn(f"ssh {USER}@{HOST}")

i = child.expect(["yes/no", "password:", pexpect.EOF, pexpect.TIMEOUT])

if i == 0:
    child.sendline("yes")
    child.expect("password:")
    child.sendline(PASSWORD)
elif i == 1:
    child.sendline(PASSWORD)
else:
    print("Connection failed.")
    exit()
child.sendline("cd nidar")
child.sendline("rm -rf images metadata debug waypoints")
child.sendline("source install/setup.bash")
child.sendline("v4l2-ctl -d /dev/lenCAM --set-ctrl=auto_exposure=3")
child.sendline("v4l2-ctl -d /dev/lenCAM --set-ctrl=gain=0")
child.sendline("v4l2-ctl -d /dev/lenCAM --set-ctrl=power_line_frequency=1")
child.sendline("v4l2-ctl -d /dev/lenCAM --set-ctrl=white_balance_automatic=0")
child.sendline("v4l2-ctl -d /dev/lenCAM --set-ctrl=white_balance_temperature=5000")
child.sendline("v4l2-ctl -d /dev/lenCAM --set-ctrl=saturation=100")
child.sendline("v4l2-ctl -d /dev/lenCAM --set-ctrl=hue=0")
child.sendline("v4l2-ctl -d /dev/lenCAM --set-ctrl=gamma=72")
child.sendline("v4l2-ctl -d /dev/lenCAM --set-ctrl=contrast=64")
child.sendline("v4l2-ctl -d /dev/lenCAM --set-ctrl=brightness=20")
child.sendline("v4l2-ctl -d /dev/lenCAM --set-ctrl=sharpness=3")
child.sendline("v4l2-ctl -d /dev/lenCAM --set-ctrl=backlight_compensation=0")
child.sendline("nohup ros2 run cleo mission_launch > mission.txt")
child.interact()

