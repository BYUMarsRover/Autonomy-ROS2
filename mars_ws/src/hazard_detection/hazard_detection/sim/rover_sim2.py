import pygame
import math

class RoverSim:
    def __init__(self, width=800, height=600):
        pygame.init()
        self.WIDTH, self.HEIGHT = width, height
        self.ROVER_WIDTH, self.ROVER_HEIGHT = 40, 20
        self.TARGET_RADIUS = 10
        self.WHITE = (255, 255, 255)
        self.GREEN = (0, 255, 0)
        self.RED = (255, 0, 0)
        self.BLUE = (0, 0, 255)
        self.BLACK = (0, 0, 0)

        self.screen = pygame.display.set_mode((self.WIDTH, self.HEIGHT))
        pygame.display.set_caption("Rover Simulation")
        self.clock = pygame.time.Clock()

        self.rover_x, self.rover_y = self.WIDTH // 2, self.HEIGHT // 2
        self.rover_angle = 0  # Facing north (positive x direction)
        self.velocity = 0
        self.angular_velocity = 0

        self.target_x, self.target_y = self.WIDTH - 100, self.HEIGHT // 2
        self.obstacles = [(300, 300, 50, 50)]
        self.running = True

    def draw_rover(self):
        rect = pygame.Rect(0, 0, self.ROVER_WIDTH, self.ROVER_HEIGHT)
        rect.center = (self.rover_x, self.rover_y)
        
        rotated_surf = pygame.Surface((self.ROVER_WIDTH, self.ROVER_HEIGHT), pygame.SRCALPHA)
        rotated_surf.fill(self.WHITE)
        pygame.draw.rect(rotated_surf, self.BLUE, (0, 0, self.ROVER_WIDTH, self.ROVER_HEIGHT))
        rotated_surf = pygame.transform.rotate(rotated_surf, -math.degrees(self.rover_angle))
        rect = rotated_surf.get_rect(center=(self.rover_x, self.rover_y))
        self.screen.blit(rotated_surf, rect.topleft)
        
        front_x = self.rover_x + (self.ROVER_WIDTH // 2) * math.cos(self.rover_angle)
        front_y = self.rover_y - (self.ROVER_WIDTH // 2) * math.sin(self.rover_angle)
        pygame.draw.circle(self.screen, self.GREEN, (int(front_x), int(front_y)), 5)

    def draw_target(self):
        pygame.draw.circle(self.screen, self.RED, (self.target_x, self.target_y), self.TARGET_RADIUS)

    def draw_obstacles(self):
        for obs in self.obstacles:
            pygame.draw.rect(self.screen, self.BLACK, obs)

    def update_rover(self):
        self.rover_angle += self.angular_velocity
        self.rover_x += self.velocity * math.cos(self.rover_angle * 2 * math.pi)
        self.rover_y -= self.velocity * math.sin(self.rover_angle * 2 * math.pi)

    def set_velocity(self, velocity, angular_velocity):
        self.velocity = velocity
        self.angular_velocity = angular_velocity

    def add_obstacle(self, x, y, width, height):
        self.obstacles.append((x, y, width, height))

    def remove_obstacles(self):
        self.obstacles = []

    def run(self):
        while self.running:
            self.screen.fill(self.WHITE)
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
            self.update_rover()
            self.draw_rover()
            self.draw_target()
            self.draw_obstacles()
            pygame.display.flip()
            self.clock.tick(30)
        pygame.quit()

if __name__ == "__main__":
    sim = RoverSim()
    sim.run()
