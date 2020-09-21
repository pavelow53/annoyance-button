from machine import Pin
from neopixel import NeoPixel
import uasyncio
import gc
import time
import network
from umqtt.simple import MQTTClient

# Settings
pin_button = 12
pin_buzzer = 4
pin_np = 13
wifi_ssid = ""
wifi_password = ""
mqtt_server = ""
mqtt_port = 8883
mqtt_user = ""
mqtt_password = ""
neopixel_animation_delay_ms = 20
neopixel_animation_spin_delay_ms = 500
neopixel_boot_color = (0, 128, 0)
neopixel_animation_spin_color = (204, 51, 255)
mqtt_subtopic_linked_alert = b"annoyance/shan/as/s/alert/"
mqtt_subtopic_linked_alive = b"annoyance/shan/as/s/alert/"
mqtt_pubtopic_alert = b"annoyance/shan/as/a/alert/"
mqtt_pubtopic_alive = b"annoyance/shan/as/a/alive/"
# mqtt_subtopic_linked_alert = b"annoyance/shan/as/a/alert/"
# mqtt_subtopic_linked_alive = b"annoyance/shan/as/a/alert/"
# mqtt_pubtopic_alert = b"annoyance/shan/as/s/alert/"
# mqtt_pubtopic_alive = b"annoyance/shan/as/s/alive/"
mqtt_pubtopic_alert_msg_ready = b"ready?"
mqtt_pubtopic_alert_msg_response = b"yes."
mqtt_pubtopic_alert_msg_start = b"start."
mqtt_pubtopic_alert_msg_ok = b"ok."
mqtt_pubtopic_alert_msg_stop = b"stop!"
mqtt_pubtopic_alive_msg_alive = b"alive?"
mqtt_pubtopic_alive_msg_response = b"yes."
connection_timeout = 5

# Program variables
state = {}
state['button_alert'] = False
state['button_check'] = False
state['alert'] = False
state['check'] = False
state['wifi'] = False
state['mqtt'] = False
state['linked'] = False
state['alert_msg_ready'] = False
state['alert_msg_response'] = False
state['alert_msg_start'] = False
state['alert_msg_ok'] = False
state['alert_msg_stop'] = False
state['alive_msg_alive'] = False
state['alive_msg_response'] = False
state['timeout'] = False

# MQTT setup
c = MQTTClient("umqtt_client", server=mqtt_server, port=mqtt_port, user=mqtt_user, password=mqtt_password, ssl=True)

# Buzzer
buzzer = Pin(pin_buzzer, Pin.OUT)
async def beep(times=1, duration=50, spacing=200):
    for _ in range(times):
        buzzer.on()
        await uasyncio.sleep_ms(duration)
        buzzer.off()
        await uasyncio.sleep_ms(spacing)

# Button
button = Pin(pin_button, Pin.IN, Pin.PULL_UP)
async def button_check():
    global state
    ctr = 0
    idle_ctr = 0
    while True:
        await uasyncio.sleep_ms(100)
        if not button.value():
            ctr = ctr + 1
            # print("Button = "+str(ctr))
        else:
            idle_ctr = idle_ctr + 1
        if idle_ctr > 2:
            if ctr >= 30:
                pass
                # await beep(1, 500)
                # state['button_check'] = True
            elif ctr >= 5:
                if not state['button_alert'] and state['alert']:
                    await beep(2, 500, 100)
                    state['alert'] = False
                elif not state['button_alert']:
                    await beep()
                    state['button_alert'] = True
            idle_ctr = 0
            ctr = 0

# Neopixel
class NeoPixelDrive():
    def __init__(self, pin_number):
        pin = Pin(pin_number, Pin.OUT) # set GPIO0 to output to drive NeoPixels
        self.np = NeoPixel(pin, 24) # create NeoPixel driver on GPIO0 for 8 pixels
        # Constants
        self.white = (255, 255, 255)
        self.red = (255, 0, 0)
        self.green = (0, 255, 0)
        self.blue = (0, 0, 255)
        self.purple = (204, 51, 255)
        self.low_white = (1, 1, 1)
        self.low_green = (0, 1, 0)
        self.low_red = (2, 0, 0)
        self.off = (0, 0, 0)

    def set_all(self, rgb):
        for i in range(24):
            self.np[i] = rgb # set the first pixel to white
        self.np.write() # write data to all pixels

    def animation_boot_blocking(self, rgb, range_l, range_h):
        for i in range(range_l, range_h):
            self.np[i] = rgb
            self.np.write()
            time.sleep_ms(neopixel_animation_delay_ms)

    async def animation_connect(self, rgb, range_l, range_h):
        for i in range(range_l, range_h):
            self.np[i] = rgb
            self.np.write()
            await uasyncio.sleep_ms(neopixel_animation_delay_ms)

    async def animation_spin(self, rgb):
        for i in range(24):
            self.np[i] = self.off
        await beep(2, 100, 100)
        for i in range(24):
            self.np[i] = rgb
            self.np.write()
            await uasyncio.sleep_ms(neopixel_animation_delay_ms)
        for i in range(24):
            self.np[i] = self.off
            self.np.write()
            await uasyncio.sleep_ms(neopixel_animation_delay_ms)
        self.set_all(self.off)

    async def animation_flash(self, rgb, n=1, duration=200, spacing=100):
        for _ in range(n):
            buzzer.on()
            self.set_all(rgb)
            await uasyncio.sleep_ms(duration)
            buzzer.off()
            self.set_all(self.off)
            await uasyncio.sleep_ms(spacing)

    async def set_alert(self):
        print("Set alert")
        state['alert_msg_stop'] = False
        t = time.time()
        while True:
            if time.time() - t > 30:
                break
            if not state['alert']:
                print("Publishing STOP.")
                c.publish(mqtt_pubtopic_alert, mqtt_pubtopic_alert_msg_stop)
                break
            # await beep(2)
            c.check_msg()
            if state['alert_msg_stop']:
                state['alert'] = False
                await self.animation_flash(neopixel_animation_spin_color, 3)
                break
            await self.animation_spin(neopixel_animation_spin_color)
            await uasyncio.sleep_ms(neopixel_animation_spin_delay_ms)

    async def set_linked(self):
        increment = 12
        value = -11
        for _ in range(20):
            value = value + increment
            for i in range(24):
                self.np[i] = (0, value, 0)
            self.np.write()
            await uasyncio.sleep_ms(50)
        for _ in range(18):
            value = value - increment
            for i in range(24):
                self.np[i] = (0, value, 0)
            self.np.write()
            await uasyncio.sleep_ms(50)
        for i in range(24):
            self.np[i] = (0, 0, 0)
        self.np.write()

    async def set_unlinked(self):
        increment = 12
        value = -11
        for _ in range(20):
            value = value + increment
            for i in range(24):
                self.np[i] = (0, 0, value)
            self.np.write()
            await uasyncio.sleep_ms(50)
        for _ in range(18):
            value = value - increment
            for i in range(24):
                self.np[i] = (0, 0, value)
            self.np.write()
            await uasyncio.sleep_ms(50)
        for i in range(24):
            self.np[i] = (0, 0, 0)
        self.np.write()

    async def set_disconnected(self):
        increment = 12
        value = -11
        for _ in range(20):
            value = value + increment
            for i in range(24):
                self.np[i] = (value, 0, 0)
            self.np.write()
            await uasyncio.sleep_ms(50)
        for _ in range(18):
            value = value - increment
            for i in range(24):
                self.np[i] = (value, 0, 0)
            self.np.write()
            await uasyncio.sleep_ms(50)
        for i in range(24):
            self.np[i] = (0, 0, 0)
        self.np.write()

npd = NeoPixelDrive(pin_np)

# WiFi
def wifi_connect():
    npd.set_all(npd.off)
    npd.animation_boot_blocking(neopixel_boot_color, 0, 6)
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print('connecting to network...')
        wlan.connect(wifi_ssid, wifi_password)
        # wlan.connect('picosys-2.4Ghz', 'F*8w5ia8ASI%0zaq')
        while not wlan.isconnected():
            pass
    npd.animation_boot_blocking(neopixel_boot_color, 6, 12)
    print('network config:', wlan.ifconfig())

# MQTT

def sub_cb(topic, msg):
    print((topic, msg))
    if topic == mqtt_subtopic_linked_alert:
        if msg == mqtt_pubtopic_alert_msg_response:
            state['alert_msg_response'] = True
        if msg == mqtt_pubtopic_alert_msg_ok:
            state['alert_msg_ok'] = True
        if msg == mqtt_pubtopic_alert_msg_stop:
            state['alert_msg_stop'] = True
        if msg == mqtt_pubtopic_alert_msg_ready:
            state['alert_msg_ready'] = True
        if msg == mqtt_pubtopic_alert_msg_start:
            state['alert_msg_start'] = True
    elif topic == mqtt_subtopic_linked_alive:
        pass

c.set_callback(sub_cb)

def mqtt_connect():
    print("MQTT connecting.")
    c.connect()
    npd.animation_boot_blocking(neopixel_boot_color, 12, 18)
    print("MQTT Connected.")
    c.subscribe(mqtt_subtopic_linked_alert)
    c.subscribe(mqtt_subtopic_linked_alive)
    npd.animation_boot_blocking(neopixel_boot_color, 18, 24)

# State machine
async def state_machine():
    while True:
        # Process buttons
        if state['button_alert']:
            state['button_alert'] = False
            state['alert'] = True
            # Set up connection to mqtt
            await npd.animation_connect(neopixel_animation_spin_color, 0, 12)
            c.publish(mqtt_pubtopic_alert, mqtt_pubtopic_alert_msg_ready) # Ready?
            state['alert_msg_response'] = False
            t = time.time()
            while not state['alert_msg_response']:
                c.check_msg()
                if time.time() - t > connection_timeout:
                    state['timeout'] = True
                    break
                await uasyncio.sleep_ms(50)
            if not state['timeout']:
                await npd.animation_connect(neopixel_animation_spin_color, 12, 24)
                c.publish(mqtt_pubtopic_alert, mqtt_pubtopic_alert_msg_start) # Start.
                state['alert_msg_ok'] = False
                t = time.time()
                while not state['alert_msg_ok']:
                    c.check_msg()
                    if time.time() - t > connection_timeout:
                        state['timeout'] = True
                        break
                    await uasyncio.sleep_ms(50)
            if not state['timeout']:
                await npd.set_alert()
        elif state['button_check']:
            state['button_check'] = False
            state['check'] = True
            # Not done yet
            await npd.set_unlinked()
        if state['alert_msg_ready']:
            print("Received READY?")
            state['alert'] = True
            state['alert_msg_ready'] = False
            state['alert_msg_start'] = False
            c.publish(mqtt_pubtopic_alert, mqtt_pubtopic_alert_msg_response) # Response
            t = time.time()
            while not state['alert_msg_start']:
                c.check_msg()
                if time.time() - t > connection_timeout:
                    state['timeout'] = True
                    break
                await uasyncio.sleep_ms(50)
            if not state['timeout']:
                c.publish(mqtt_pubtopic_alert, mqtt_pubtopic_alert_msg_ok) # Response
                await npd.set_alert()
        
        if state['timeout']:
            state['alert'] = False
            state['alert_msg_ready'] = False
            state['alert_msg_start'] = False
            await npd.set_disconnected()
            state['timeout'] = False

        # Sync mqtt
        # print("Checkin'")
        c.check_msg()
        await uasyncio.sleep_ms(50)

# Main loop
async def main():
    wifi_connect()
    mqtt_connect()
    await uasyncio.sleep_ms(3000)
    npd.set_all(npd.off)

    uasyncio.create_task(button_check())
    uasyncio.create_task(state_machine())
    print("Free memory: "+str(gc.mem_free()))
    while True:
        await uasyncio.sleep_ms(60000)

uasyncio.run(main())