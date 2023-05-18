import pygame
from dataclasses import dataclass
import time
import numpy as np
from math import cos, sin, radians, pi


@dataclass
class _Time:
    delta_time: float = 0
    prev_time: float = time.time()

    def update(self) -> None:
        self.delta_time = time.time() - self.prev_time
        self.prev_time = time.time()

    @property
    def fps(self) -> float:
        if self.delta_time:
            return 1/self.delta_time
        return 0


Time = _Time()


@dataclass
class Robot:
    Time: _Time
    size: pygame.Vector2
    color: pygame.Color
    transform: pygame.Vector2 = pygame.Vector2(0, 0)
    angle: float = 0
    velocity: pygame.Vector2 = pygame.Vector2(0, 0)
    set_velocity: pygame.Vector2 = pygame.Vector2(0, 0)
    angular_velocity: float = 0
    set_angular_velocity: float = 0
    rect: pygame.Rect = pygame.Rect(0, 0, 0, 0)
    r: float = 1
    R: float = 1
    w1: float = 0
    w2: float = 0
    w3: float = 0
    w4: float = 0
    speed_ratio1: float =  1
    speed_ratio2: float = 1
    speed_ratio3: float = 1
    speed_ratio4: float = 1

    @property
    def theta(self):
        return radians(self.angle)

    @property
    def surface(self) -> pygame.Surface:
        surface = pygame.Surface(self.size)
        surface.set_colorkey((0, 0, 0))
        surface.fill(self.color)
        surface = pygame.transform.rotate(surface, self.angle)
        self.rect = surface.get_rect()
        return surface

    def blit(self, screen: pygame.Surface):
        screen.blit(self.surface, self.transform - self.rect.center)

    def move_motor(self):
        array = [[-sin(self.theta+(1*pi/4)), cos(self.theta+(1*pi/4)), 1/2*self.R],
                 [-sin(self.theta+(3*pi/4)), cos(self.theta+(3*pi/4)), 1/2*self.R],
                 [-sin(self.theta+(5*pi/4)), cos(self.theta+(5*pi/4)), 1/2*self.R],
                 [-sin(self.theta+(7*pi/4)), cos(self.theta+(7*pi/4)), 1/2*self.R]]
        array = np.array(array)
        speed = np.array([[self.w1 * self.speed_ratio1],
                          [self.w2 * self.speed_ratio2],
                          [self.w3 * self.speed_ratio3],
                          [self.w4 * self.speed_ratio4]])
        result = (self.r/2) * np.matmul(array.transpose(), speed)
        self.set_velocity.x, self.set_velocity.y, self.set_angular_velocity =\
            result.transpose()[0]

    def update(self):
        self.move_motor()
        multiplier = 2 * self.Time.delta_time
        multiplier = pygame.math.clamp(multiplier, 0, 1)
        self.velocity = self.velocity.lerp(self.set_velocity, multiplier)
        self.transform += self.velocity * self.Time.delta_time
        self.angular_velocity = pygame.math.lerp(self.angular_velocity,
                                                 self.set_angular_velocity,
                                                 multiplier)
        self.angle += self.angular_velocity * self.Time.delta_time
