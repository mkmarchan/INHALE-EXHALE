import os, sys, inspect, thread, time, math

src_dir = os.path.dirname(inspect.getfile(inspect.currentframe()))
# Windows and Linux
# arch_dir = '../lib/x64' if sys.maxsize > 2 ** 32 else '../lib/x86'
# Mac
arch_dir = os.path.abspath(os.path.join(src_dir, '../lib'))

sys.path.insert(0, os.path.abspath(os.path.join(src_dir, arch_dir)))

import Leap
import OSC


class CustomBox:
    def __init__(self, min_x, max_x, min_y, max_y, min_z, max_z):
        self.min_x = min_x
        self.max_x = max_x
        self.min_y = min_y
        self.max_y = max_y
        self.min_z = min_z
        self.max_z = max_z

    def scale(self, vector):
        return Leap.Vector(
            (vector.x - self.min_x) / (self.max_x - self.min_x),
            (vector.y - self.min_y) / (self.max_y - self.min_y),
            (vector.z - self.min_z) / (self.max_z - self.min_z)
        )


class Listener(Leap.Listener):
    def __init__(self, client, i_box):
        super(Listener, self).__init__()
        self.client = client
        self.i_box = i_box
        self.last_frame_triggered = None
        self.down_flag = False
        self.last_frame_down = None
        self.last_double_down = None

    def on_connect(self, controller):
        print "Connected"
        sys.stdout.flush()

    def on_frame(self, controller):
        frame = controller.frame()
        hand_list = frame.hands
        left_hand = None
        right_hand = None
        has_left = False
        has_right = False
        if len(hand_list) > 0:
            for hand in hand_list:
                left_hand = hand if hand.is_left else left_hand
                has_left = True if hand.is_left else has_left
                right_hand = hand if hand.is_right else right_hand
                has_right = True if hand.is_right else has_right


        msg = OSC.OSCMessage()
        # message name (0)
        msg.setAddress("/LeapMotion")

        # does left exist? (1)
        msg.append(1 if has_left else 0)

        scaled_palm_pos = self.i_box.scale(left_hand.palm_position) if has_left else None

        # x position of left palm (2)
        msg.append(scaled_palm_pos.x if has_left else -5347)
        # y position of left palm (3)
        msg.append(scaled_palm_pos.y if has_left else -5347)
        # z position of left palm (4)
        msg.append(scaled_palm_pos.z if has_left else -5347)
        # roll of left palm (5)
        msg.append((math.atan2(left_hand.palm_normal.y, left_hand.palm_normal.x)) if has_left else -5347)
        # pitch of left palm (6)
        msg.append((math.atan2(left_hand.palm_normal.y, left_hand.palm_normal.z)) if has_left else -5347)
        # yaw of left palm (7)
        msg.append((math.atan2(left_hand.direction.z, left_hand.direction.x)) if has_left else -5347)
        # x velocity of left palm (8)
        msg.append(left_hand.palm_velocity.x if has_left else -5347)
        # grab strength (9)
        msg.append(left_hand.grab_strength if has_left else -5347)
        # switch synth (10)
        switch = 0
        if has_left and left_hand.grab_strength > 0.8:
	    if  left_hand.palm_velocity.y < -1500 and abs(left_hand.palm_velocity.x) < 500:
                if self.down_flag and self.last_frame_down and frame.timestamp - self.last_frame_down.timestamp < 2000000:
                    print "confirm"
		    sys.stdout.flush()
                    switch = 2
                    self.down_flag = False
                    self.last_frame_down = None
		    self.last_double_down = frame
                elif self.down_flag:
                    self.down_flag = False
                    self.last_frame_down = frame
                else:
                    self.last_frame_down = frame
            elif abs(left_hand.palm_velocity.x) > 2000 and\
                (not self.last_frame_triggered or (frame.timestamp - self.last_frame_triggered.timestamp > 500000)):
                if left_hand.palm_velocity.x < 0:
                    switch = 1
                    print "switching left"
                else:
                    switch = -1
                    print "switching right"
                sys.stdout.flush()
                self.last_frame_triggered = frame
            elif left_hand.palm_velocity.y > 0 and self.last_frame_down and frame.timestamp - self.last_frame_down.timestamp < 2000000\
              and (not self.last_double_down or frame.timestamp - self.last_double_down.timestamp > 2000000):
                # print left_hand.palm_velocity.y
		sys.stdout.flush()
                self.down_flag = True

        msg.append(switch)

        # does right exist? (11)
        msg.append(1 if has_right else 0)

        scaled_palm_pos = self.i_box.scale(right_hand.palm_position) if has_right else None

        # x position of left palm (12)
        msg.append(scaled_palm_pos.x if has_right else -5347)
        # y position of left palm (13)
        msg.append(scaled_palm_pos.y if has_right else -5347)
        # z position of left palm (14)
        msg.append(scaled_palm_pos.z if has_right else -5347)
        # roll of left palm (15)
        msg.append((math.atan2(right_hand.palm_normal.y, right_hand.palm_normal.x)) if has_right else -5347)
        # pitch of left palm (16)
        msg.append((math.atan2(right_hand.palm_normal.y, right_hand.palm_normal.z)) if has_right else -5347)
        # yaw of left palm (17)
        msg.append((math.atan2(right_hand.direction.z, right_hand.direction.x)) if has_right else -5347)
        # x velocity of left palm (18)
        msg.append(right_hand.palm_velocity.x if has_right else -5347)
        # grab strength (19)
        msg.append(right_hand.grab_strength if has_right else -5347)

        self.client.send(msg)


def main():
    # setup variables
    ip = "127.0.0.1"  # OSC destination hostname
    port = 5347  # OSC destination port
    client = OSC.OSCClient()
    client.connect((ip, port))
    i_box = CustomBox(-250, 250, 250, 500, -250, 250)  # interaction box size

    print "Connecting..."
    sys.stdout.flush()
    listener = Listener(client, i_box)
    controller = Leap.Controller()
    controller.add_listener(listener)

    print "Press Enter to quit..."
    sys.stdout.flush()
    try:

        sys.stdin.readline()
    except KeyboardInterrupt:
        pass
    finally:
        print "Disconnecting..."
        sys.stdout.flush()
        controller.remove_listener(listener)


if __name__ == "__main__":
    main()
