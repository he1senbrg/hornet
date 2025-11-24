import pygame
pygame.init()
pygame.joystick.init()
print("Joysticks detected:", pygame.joystick.get_count())
for i in range(pygame.joystick.get_count()):
    js = pygame.joystick.Joystick(i)
    js.init()
    print("Joystick", i, "name:", js.get_name())
