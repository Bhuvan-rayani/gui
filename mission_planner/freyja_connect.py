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
child.sendline("nohup ros2 run freyja freyja.launch.py > freyja.txt")
child.interact()

