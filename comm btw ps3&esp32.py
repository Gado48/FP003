import pygame
import socket
import sys

def main():
    esp32_ip = "192.168.1.4"
    esp32_port = 8080

    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((esp32_ip, esp32_port))
    except socket.error as e:
        print(f"Socket error: {e}")
        sys.exit(1)

    try:
        pygame.joystick.init()
        joystick_count = pygame.joystick.get_count()

        if joystick_count == 0:
            print("No joysticks found.")
            return

        joysticks = [pygame.joystick.Joystick(x) for x in range(joystick_count)]
        pygame.init()

        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    break
                if event.type == pygame.JOYBUTTONDOWN:
                    button_pressed = event.button
                    send_data(s, f"Button Pressed: {button_pressed}")
                if event.type == pygame.JOYAXISMOTION:
                    axis = event.axis
                    value = event.value
                    if axis == 0:
                        if value > 0.5:
                            send_data(s, "Joystick Moved: Right")
                        elif value < -0.5:
                            send_data(s, "Joystick Moved: Left")
                    elif axis == 1:
                        if value > 0.5:
                            send_data(s, "Joystick Moved: Back")
                        elif value < -0.5:
                            send_data(s, "Joystick Moved: Forward")
    except KeyboardInterrupt:
        print("User terminated the program.")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        s.close()
        pygame.quit()

def send_data(socket, data):
    try:
        socket.send(data.encode())
    except socket.error as e:
        print(f"Socket error while sending data: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()