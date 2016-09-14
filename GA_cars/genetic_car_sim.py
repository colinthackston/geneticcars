import math
import random
import struct
import pygame
from ast import literal_eval
from pygame.locals import *
from random import randint
import Box2D
from Box2D.b2 import *
from Box2D import *

print pygame.__file__
pygame.init()

#bounded variable sizes to prevent cars that would never succeed
generation_count=0
groundPieceWidth = 1.5
groundPieceHeight = 0.15

chassisMaxAxis = 1.5
chassisMinAxis = 0.2
chassisMinDensity = 25
chassisMaxDensity = 125

wheelMaxRadius = 1.50
wheelMinRadius = 0.2
wheelMaxDensity = 50
wheelMinDensity = 5
motorSpeed = 100

#set gravity
gravity = b2Vec2(0.0, -9.81)
doSleep = True

# print help(world)
start_position = b2Vec2(8, 2)

max_health = 100

import random

# class for the car that gets all genetic algorithm-ed
#stores all the functions for returning car data
class car_info:
	#inits the car
    def __init__(self):
        self.wheel_count = 2  # default
        self.wheel_radius = [0] * self.wheel_count
        self.wheel_density = [0] * self.wheel_count
        self.wheel_vertex = [0] * self.wheel_count
        self.chassis_density = 1  # default
        self.vertex_list = [0] * 8

    def get_wheel_count(self):
        return self.wheel_count

    def get_wheel_radius(self):
        return self.wheel_radius

    def get_wheel_density(self):
        return self.wheel_density

    def get_wheel_vertex(self):
        return self.wheel_vertex

    def get_chassis_density(self):
        return self.chassis_density

    def get_vertex_list(self):
        return self.vertex_list

    def set_wheel_count(self, val):
        self.wheel_count = val

    def set_wheel_radius(self, val_list):
        self.wheel_radius = val_list

    def set_wheel_density(self, val_list):
        self.wheel_density = val_list

    def set_wheel_vertex(self, val_list):
        self.wheel_vertex = val_list

    def set_chassis_density(self, val):
        self.chassis_density = val

    def set_vertex_list(self, val_list):
        self.vertex_list = val_list


    def car_to_binary(self):
        car_as_binary_num=[]
        wheels=bin(self.get_wheel_count())
        wheel_radius_list=self.get_wheel_radius()
        wheelR1=format((int((wheel_radius_list[0])*100)),"#011b")

        wheelR2=format((int((wheel_radius_list[1]) * 100)),"#011b")


        wheel_density_list= self.get_wheel_density()
        wheelD1=format((int (wheel_density_list[0] * 10)),"#011b")


        wheelD2=format((int(wheel_density_list[1] * 10)),"#011b")


        wheel_vertex_list= (self.get_wheel_vertex())
        wheelV1= format((wheel_vertex_list[0]), "#011b")
        wheelV2= format((wheel_vertex_list[1]), "#011b")
        chassis_density= format((int(self.get_chassis_density())),"#011b")
        vertex_list=(self.get_vertex_list())
        result=wheelR1+wheelR2+wheelD1+wheelD2+wheelV1+wheelV2+chassis_density
        for x in range(8):
            temp=vertex_list[x]
            result=result+format((int((temp[0]*100))), "#011b") + format((int((temp[1]*100))),"#011b")
        return result



#class for infor about car
#capable of actually initing the car and making it drive
class car:
    def __init__(self, world, random=True, car_def=None):
        global motorSpeed, gravity, groundPieceWidth, groundPieceHeight, chassisMaxAxis, chassisMinAxis, chassisMinDensit, chassisMaxDensity, wheelMaxRadius, wheelMinRadius, wheelMaxDensity, wheelMinDensit
        self.world = world
        if random:
            self.car_def = self.make_random_car()
        else:
            self.car_def = car_def

        #starts the car at 0, alive, and creates the chassis
        self.alive = True;
        self.velocityIndex = 0;
        self.chassis = self.create_chassis(self.car_def.vertex_list, self.car_def.chassis_density)
        self.wheels = []

        #makes the wheels
        for i in range(self.car_def.wheel_count):
            self.wheels.append(self.create_wheel(self.car_def.wheel_radius[i], self.car_def.wheel_density[i]))
        carmass = self.chassis.mass
        for i in range(self.car_def.wheel_count):
            carmass += self.wheels[i].mass

        carmass = 2 + 1
        # better: theres a getMassData method for b2Body - check that!
        self.torque = []
        #sets the car wheel speed in relation to gravity (torque)
        for i in range(self.car_def.wheel_count):
            self.torque.append(carmass * -gravity.y / self.car_def.wheel_radius[i])

        self.joint_def = b2RevoluteJointDef()

        #creates the vertecies for the car
        for i in range(self.car_def.wheel_count):
            randvertex = self.chassis.vertex_list[self.car_def.wheel_vertex[i]]
            self.joint_def.localAnchorA.Set(randvertex.x, randvertex.y)
            self.joint_def.localAnchorB.Set(0, 0)
            self.joint_def.maxMotorTorque = self.torque[i]
            self.joint_def.motorSpeed = -motorSpeed
            self.joint_def.enableMotor = True
            self.joint_def.collideConnected = False
            self.joint_def.bodyA = self.chassis
            self.joint_def.bodyB = self.wheels[i]
            joint = self.world.CreateJoint(self.joint_def)

        # print "->",self.chassis.fixtures[0].type

    #makes a random car for migration and mutation purposes
    def make_random_car(self):
        global motorSpeed, gravity, groundPieceWidth, groundPieceHeight, chassisMaxAxis, chassisMinAxis, chassisMinDensit, chassisMaxDensity, wheelMaxRadius, wheelMinRadius, wheelMaxDensity, wheelMinDensit

        random_car = car_info()

        wheel_radius_values = []
        wheel_density_values = []
        vertex_list = []
        wheel_vertex_values = []

        #creates a random wheel radius and density
        for i in range(random_car.get_wheel_count()):
            wheel_radius_values.append(random.random() * wheelMaxRadius + wheelMinRadius)
            wheel_density_values.append(random.random() * wheelMaxDensity + wheelMinDensity)

        #creates a random chassis
        vertex_list.append(b2Vec2(random.random() * chassisMaxAxis + chassisMinAxis, 0))
        vertex_list.append(b2Vec2(random.random() * chassisMaxAxis + chassisMinAxis,
                                  random.random() * chassisMaxAxis + chassisMinAxis))
        vertex_list.append(b2Vec2(0, random.random() * chassisMaxAxis + chassisMinAxis))
        vertex_list.append(b2Vec2(-random.random() * chassisMaxAxis - chassisMinAxis,
                                  random.random() * chassisMaxAxis + chassisMinAxis))
        vertex_list.append(b2Vec2(-random.random() * chassisMaxAxis - chassisMinAxis, 0))
        vertex_list.append(b2Vec2(-random.random() * chassisMaxAxis - chassisMinAxis,
                                  -random.random() * chassisMaxAxis - chassisMinAxis))
        vertex_list.append(b2Vec2(0, -random.random() * chassisMaxAxis - chassisMinAxis))
        vertex_list.append(b2Vec2(random.random() * chassisMaxAxis + chassisMinAxis,
                                  -random.random() * chassisMaxAxis - chassisMinAxis))


        index_left = [i for i in range(8)]
        # print index_left
        for i in range(random_car.get_wheel_count()):
            index_of_next = int(random.random() * (len(index_left) - 1))
            # print index_of_next
            wheel_vertex_values.append(index_left[index_of_next])
            # remove the last used index from index_left
            index_left = index_left[:index_of_next] + index_left[index_of_next + 1:]

        #set all attributes
        random_car.set_vertex_list(vertex_list)
        random_car.set_wheel_radius(wheel_radius_values)
        random_car.set_wheel_density(wheel_density_values)
        random_car.set_wheel_vertex(wheel_vertex_values)
        random_car.set_chassis_density(random.random() * chassisMaxDensity + chassisMinDensity)

        return random_car

    #creates a wheel
    def create_wheel(self, radius, density):
        body_def = bodyDef() #this line has an error, ignore it and it will still run
        body_def.type = b2_dynamicBody
        body_def.position.Set(start_position.x, start_position.y)
        body = self.world.CreateBody(body_def)
        fix_def = b2FixtureDef()
        fix_def.shape = b2CircleShape(radius=radius)
        fix_def.density = density
        fix_def.friction = 1
        fix_def.restitution = 0.2
        fix_def.filter.groupIndex = -1
        body.CreateFixture(fix_def)

        return body

    #creates the chassis parts
    def create_chassis_part(self, body, vertex1, vertex2, density):
        vertex_list = []
        vertex_list.append(vertex1)
        vertex_list.append(vertex2)
        vertex_list.append(b2Vec2(0, 0))
        fix_def = b2FixtureDef()
        fix_def.shape = b2PolygonShape()
        fix_def.density = density
        fix_def.friction = 10
        fix_def.restitution = 0.0
        fix_def.filter.groupIndex = -1

        fix_def.shape = b2PolygonShape(vertices=vertex_list)
        body.CreateFixture(fix_def)

    #appends all the chassis parts together into a chassis
    def create_chassis(self, vertex_list, density):
        body_def = b2BodyDef()
        body_def.type = b2_dynamicBody;
        body_def.position.Set(start_position.x, start_position.y)  # start position of the car
        body = self.world.CreateBody(body_def)
        for i in range(len(vertex_list)):
            self.create_chassis_part(body, vertex_list[i], vertex_list[(i + 1) % 8], density)
        body.vertex_list = vertex_list
        return body

    def get_car_chassis(self):
        return self.chassis

    def get_car_wheels(self):
        return self.wheels


#creates the terrian
class terrain:
    def __init__(self, world):
        self.world = world

    def create_floor(self):
        maxFloorTiles = 200
        last_tile = None
        tile_position = b2Vec2(-1, 0)
        floor_tiles = []
        random.seed(random.randint(1, 39478534))
        for k in range(maxFloorTiles):
            last_tile = self.create_floor_tile(tile_position, (random.random() * 3 - 1.5) * 1.2 * k / maxFloorTiles)
            floor_tiles.append(last_tile)
            last_fixture = last_tile.fixtures

            if last_fixture[0].shape.vertices[3] == b2Vec2(0, 0):
                last_world_coords = last_tile.GetWorldPoint(last_fixture[0].shape.vertices[0])
            else:
                last_world_coords = last_tile.GetWorldPoint(last_fixture[0].shape.vertices[3])
            tile_position = last_world_coords

        return floor_tiles

    def create_floor_tile(self, position, angle):
        # print "creating next tile at position: ",position
        global motorSpeed, gravity, groundPieceWidth, groundPieceHeight, chassisMaxAxis, chassisMinAxis, chassisMinDensit, chassisMaxDensity, wheelMaxRadius, wheelMinRadius, wheelMaxDensity, wheelMinDensit
        body_def = b2BodyDef()
        # body_def.position.Set(position.x, position.y)
        body_def.position = position
        body = self.world.CreateBody(body_def)
        fix_def = b2FixtureDef()
        fix_def.shape = b2PolygonShape()
        fix_def.friction = 0.5
        coords = []
        coords.append(b2Vec2(0, 0))
        coords.append(b2Vec2(0, groundPieceHeight))
        coords.append(b2Vec2(groundPieceWidth, groundPieceHeight))
        coords.append(b2Vec2(groundPieceWidth, 0))
        newcoords = self.rotate_floor_tile(coords, angle)

        fix_def.shape = b2PolygonShape(vertices=newcoords)  # setAsArray alt

        body.CreateFixture(fix_def)

        return body

    def rotate_floor_tile(self, coords, angle):
        newcoords = []
        for k in range(len(coords)):
            nc = b2Vec2(0, 0)
            nc.x = math.cos(angle) * (coords[k].x) - math.sin(angle) * (coords[k].y)
            nc.y = math.sin(angle) * (coords[k].x) + math.cos(angle) * (coords[k].y)
            newcoords.append(nc)
        return newcoords


class car_data:
    global start_position, max_health

    def __init__(self, chassis, wheels, car_def, xy_pos=[0, 0], linear_vel=0):
        self.xy_pos = xy_pos  # [x,y]
        self.linear_vel = linear_vel
        self.health = max_health
        self.isDead = False
        self.chassis = chassis
        self.wheels = wheels
        self.max_dist = 0
        self.car_def = car_def

    def kill_it(self):
        self.health = 0
        self.isDead = True

    def getHealth(self):
        return self.health

    def isDead(self):
        return self.isDead

    def dcr_health(self):
        self.health -= 2

    def get_vel(self):
        return self.linear_vel

    def get_pos_x(self):
        return self.xy_pos[0]

    def get_pos(self):
        return self.xy_pos

    def set_pos_and_vel(self, pos, vel):
        if not self.isDead:
            self.xy_pos = pos
            self.linear_vel = vel
            self.update_health()
            self.update_max_dist()

    def update_health(self):
        if self.linear_vel < 0.001:
            self.dcr_health()
            if self.health <= 0:
                self.kill_it()

    def print_info(self):
        if not self.isDead:
            print "Velocity:", self.linear_vel, " Position:", self.xy_pos, " Health:", self.health
        else:
            print "Dead"

    def update_max_dist(self):
        self.max_dist = self.xy_pos[0] - start_position.x


import __builtin__


# class do_stuff(Framework): #uncomment for framework stuff
class do_stuff():
    generations = 0
    global motorSpeed, generation_count,gravity, groundPieceWidth, groundPieceHeight, chassisMaxAxis, chassisMinAxis, chassisMinDensit, chassisMaxDensity, wheelMaxRadius, wheelMinRadius, wheelMaxDensity, wheelMinDensit

    def __init__(self):
        self.world = b2World(gravity=(0, -9.81), doSleep=True)

        self.population_size = 20

        self.killed = 0

        t = terrain(self.world)
        self.terrain = t.create_floor()

        self.population = []  # array of list of [chassis,wheels]
        self.population_data = []  # array of car_data objetcs

        self.create_generation_1()
        self.leader_coors = [0, 0]
        self.leader = self.population[0][0]  # chassis of 1st car

        self.draw_any()


    def draw_any(self):
        # type = 1 means polygon, type = 2 means circle

        x_offset = 0
        y_offset = 0
        prev_y = 0
        offset_value = 5

        PPM = 30.0  # pixels per meter
        TARGET_FPS = 60
        TIME_STEP = 1.0 / TARGET_FPS
        SCREEN_WIDTH, SCREEN_HEIGHT = 640, 480
        running = True
        screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT), 0, 32)
        pygame.display.set_caption('Rolling Rolling Rolling')
        clock = pygame.time.Clock()

        colors = {staticBody: (136, 150, 200, 255), dynamicBody: (127, 127, 127, 255)} #ignore the error on this line
        leader_coors = self.leader_coors

        def my_draw_circle(circle, body, fixture):
            global wheelMinDensity
            position = body.transform * circle.pos * PPM

            y_offset = ((self.leader.worldCenter.y) * 70)
            if y_offset < -300:
                y_offset = -300
            if y_offset > 300:
                y_offset = 300

            position = (
                position[0] - self.leader.worldCenter.x * 30 + 350, SCREEN_HEIGHT - position[1] + y_offset * 0.5 - 200)


            center = [int(x) for x in position]  # just (x,y) coor

            center_s = [int(circle.radius * PPM),
                        int(circle.radius * PPM)]  # this is for drawing on the new surface we create below

            s = pygame.Surface(
                (50, 50))  # create a surface just enough for the wheel radius , too big will cause the sim. to lag
            s.set_alpha(100)  # transparancy value
            s.fill((255, 255, 255))  # fill the screen
            s.set_colorkey((255, 255, 255))  # comment this to see how screen blit works

            pygame.draw.circle(s, (38, 192, 90), center_s, int(circle.radius * PPM),
                               0)  # draw a circle on the new screen we created

            t = body.transform
            axis = b2Mul(t.q, b2Vec2(10.0, 25.0))

            pygame.draw.aaline(s, (255, 0, 0), center_s,
                               (center_s[0] - circle.radius * axis[0], center_s[1] + circle.radius * axis[1]))

            screen.blit(s, (position[0] - int(circle.radius * PPM), position[1] - int(circle.radius * PPM)))

        b2CircleShape.draw = my_draw_circle

        def my_draw_polygon(polygon, body, fixture):
            y_offset = ((self.leader.worldCenter.y) * 70)
            if y_offset < -300:
                y_offset = -300
            if y_offset > 300:
                y_offset = 300
            # print y_offset
            vertices = [(body.transform * v) * PPM for v in polygon.vertices]
            vertices = [(v[0] - self.leader.worldCenter.x * 30 + 350, SCREEN_HEIGHT - v[1] + y_offset * 0.5 - 200) for v
                        in vertices]

            pygame.draw.polygon(screen, colors[body.type], vertices)

        polygonShape.draw = my_draw_polygon

        generation_count=0
        record_distance=[0,0]
        list_of_averages=[]
        highest_generation_average=[0,0]
        while running:
            #actually runs the program
            self.update_car_data()
            self.update_leader()

            #restarts the generations
            if self.killed == self.population_size:
                self.sort_distance2()

                #updates generation count
                generation_count=generation_count+1
                if (self.population_data[0].chassis.worldCenter.x > record_distance[0]):
                    record_distance[0]=self.population_data[0].chassis.worldCenter.x
                    record_distance[1]=generation_count

                generation_average_distance=0.0

                #computes the average distance
                for i in range (int (self.population_size *.2)):
                    generation_average_distance=generation_average_distance + self.population_data[i].chassis.worldCenter.x
                generation_average_distance1=generation_average_distance/(self.population_size *.2)

                list_of_averages.append(generation_average_distance1)

                #updates the generation average
                if (generation_average_distance1>highest_generation_average[0]):
                    highest_generation_average[0]=generation_average_distance1
                    highest_generation_average[1]=generation_count

                print "Current generation number: ", generation_count
                print "Generation average_distance: ", generation_average_distance1
                print "Record distance: ", record_distance[0]
                print "Occured in generation: ", record_distance[1]
                print "Record average", highest_generation_average[0]
                print "Occured in generation: ", highest_generation_average[1]
                print list_of_averages
                self.next_generation()
            # Check the event queue

            screen.fill((90, 23, 100, 100))
            # 229,153,153,255

            # Draw the world
            for body in self.world.bodies:
                for fixture in body.fixtures:
                    fixture.shape.draw(body, fixture)

                    # Make Box2D simulate the physics of our world for one step.
            self.world.Step(TIME_STEP, 10, 10)

            # Flip the screen and try to keep at the target FPS
            pygame.display.flip()
            clock.tick(TARGET_FPS)

    #finds the leader so the camera can follow it
    def update_leader(self):
        sorted_data = sorted(self.population_data, key=lambda x: x.max_dist)
        for data in sorted_data:
            if not data.isDead:
                self.leader = data.chassis

    #steps the generation
    def Step(self, settings):
        super(do_stuff, self).Step(settings)

        self.update_car_data()

        if self.killed == self.population_size:
            self.next_generation()

    #starts the whole thing off
    def start(self):
        while True:
            self.update_car_data()
            if self.killed == self.population_size:
                self.next_generation()

    def update_car_data(self):
        for index, cars in enumerate(self.population_data):
            if not cars.isDead:
                cars.set_pos_and_vel([self.population[index][0].position.x, self.population[index][0].position.y],
                                     self.population[index][0].linearVelocity.x)
                if cars.isDead:
                    for wheel in self.population[index][1]:
                        if wheel:
                            self.world.DestroyBody(wheel)  # remove wheels
                    self.world.DestroyBody(self.population[index][0])  # remove chassis
                    self.population[index] = None
                    self.killed += 1  # turn this on only after all the mate,mutate methods work
                    print "purged so far;", self.killed

    #bubble sort the population by distance
    def sort_distance2(self):
        for i in range(self.population_size):
            for j in range((self.population_size-1-i)):
                x=self.population_data[j].chassis.worldCenter.x
                y=self.population_data[j+1].chassis.worldCenter.x
                if (x>y):
                    self.population_data[j].chassis.worldCenter.x , self.population_data[j+1].chassis.worldCenter.x = self.population_data[j+1].chassis.worldCenter.x, self.population_data[j].chassis.worldCenter.x
        self.population_data.reverse()


	#checks for duplicates
    def check_dup(self, parent_pair, mates_list):
        dup = False
        if parent_pair in mates_list or parent_pair[::-1] in mates_list:
            dup = True
        return dup

    #finds where the parents came from
    def get_parent_index(self, mates_list):
        parent1_index = random.randint(0, self.population_size - 1)
        parent2_index = random.randint(0, self.population_size - 1)
        while parent2_index == parent1_index and not self.check_dup([parent1_index, parent2_index], mates_list):
            parent1_index = random.randint(0, self.population_size - 1)
            parent2_index = random.randint(0, self.population_size - 1)
        return [parent1_index, parent2_index]

    #same random car function from before, just now accessable here
    def make_random_car(self):
        global motorSpeed, gravity, groundPieceWidth, groundPieceHeight, chassisMaxAxis, chassisMinAxis, chassisMinDensit, chassisMaxDensity, wheelMaxRadius, wheelMinRadius, wheelMaxDensity, wheelMinDensit

        random_car = car_info()

        wheel_radius_values = []
        wheel_density_values = []
        vertex_list = []
        wheel_vertex_values = []

        for i in range(random_car.get_wheel_count()):
            wheel_radius_values.append(random.random() * wheelMaxRadius + wheelMinRadius)
            wheel_density_values.append(random.random() * wheelMaxDensity + wheelMinDensity)

        vertex_list.append(b2Vec2(random.random() * chassisMaxAxis + chassisMinAxis, 0))
        vertex_list.append(b2Vec2(random.random() * chassisMaxAxis + chassisMinAxis,
                                  random.random() * chassisMaxAxis + chassisMinAxis))
        vertex_list.append(b2Vec2(0, random.random() * chassisMaxAxis + chassisMinAxis))
        vertex_list.append(b2Vec2(-random.random() * chassisMaxAxis - chassisMinAxis,
                                  random.random() * chassisMaxAxis + chassisMinAxis))
        vertex_list.append(b2Vec2(-random.random() * chassisMaxAxis - chassisMinAxis, 0))
        vertex_list.append(b2Vec2(-random.random() * chassisMaxAxis - chassisMinAxis,
                                  -random.random() * chassisMaxAxis - chassisMinAxis))
        vertex_list.append(b2Vec2(0, -random.random() * chassisMaxAxis - chassisMinAxis))
        vertex_list.append(b2Vec2(random.random() * chassisMaxAxis + chassisMinAxis,
                                  -random.random() * chassisMaxAxis - chassisMinAxis))

        index_left = [i for i in range(8)]
        # print index_left
        for i in range(random_car.get_wheel_count()):
            index_of_next = int(random.random() * (len(index_left) - 1))
            # print index_of_next
            wheel_vertex_values.append(index_left[index_of_next])
            # remove the last used index from index_left
            index_left = index_left[:index_of_next] + index_left[index_of_next + 1:]

        # now, setting all values (these are all the attibutes required to completely describe a car)
        random_car.set_vertex_list(vertex_list)
        random_car.set_wheel_radius(wheel_radius_values)
        random_car.set_wheel_density(wheel_density_values)
        random_car.set_wheel_vertex(wheel_vertex_values)
        random_car.set_chassis_density(random.random() * chassisMaxDensity + chassisMinDensity)

        return random_car

    # #mutate for binary car
    # def mutate(self,binary_string):
    #     #mutatation_rate=.05
    #     new_gene=randint(0,100)
    #     if new_gene<=5:
    #         flipped_chromomsome=randint(0,252)
    #
    #         if binary_string[flipped_chromomsome]=='-' or binary_string[flipped_chromomsome]=='b':
    #             flipped_chromomsome+=3
    #         if binary_string[flipped_chromomsome] == '1':
    #             binary_string[flipped_chromomsome]=='0'
    #         else:
    #             binary_string[flipped_chromomsome]=='1'
    #     return binary_string



    #crossover function
    #only works for binary cars
    # def evolve(self, parents):
    #
    #     parents = [self.population_data[parents[0]].car_def, self.population_data[parents[1]].car_def]
    #     child = car_info()
    #     child_car_binary=""
    #
    #     if len(parents[0].car_to_binary()) > len(parents[1].car_to_binary()):
    #         counter=len(parents[1].car_to_binary())
    #     else:
    #         counter=len(parents[0].car_to_binary())
    #     for x in range(counter):
    #         if bool(random.getrandbits(1)):
    #             temp=parents[0].car_to_binary()
    #             child_car_binary+=temp[x]
    #         else:
    #             temp=parents[1].car_to_binary()
    #             child_car_binary+=temp[x]
    #     # for x in range(len(child_car_binary)-1):
    #     #     if child_car_binary[x]=='b' and child_car_binary[x+1]== 'b':
    #     #         child_car_binary[x+1]=='0'
    #     child_car_binary=self.mutate(child_car_binary)
    #
    #     return child_car_binary

    #mutate function for normal cars
    #if the probability is high enough, the car will mutate
    #creates the a random car and randomly swaps a from it into the new car
    def mutate2(self,car):
        random_temp_car=self.make_random_car()
        probability=randint(0,100)
        if probability>94:
            if probability==95:
                if bool(random.getrandbits(1)):
                    car.wheel_radius[0]=random_temp_car.wheel_radius[0]
                else:
                    car.wheel_radius[1]=random_temp_car.wheel_radius[1]

            if probability==96:
                if bool(random.getrandbits(1)):
                    car.wheel_vertex[0]=random_temp_car.wheel_vertex[0]
                else:
                    car.wheel_vertex[1]=random_temp_car.wheel_vertex[1]

            if probability==97:
                if bool(random.getrandbits(1)):
                    car.wheel_density[0]=random_temp_car.wheel_density[0]
                else:
                    car.wheel_density[1]=random_temp_car.wheel_density[1]

            if probability==98:
                car.chassis_density=random_temp_car.chassis_density

            if probability==99:
                mutated_gene=random.randint(0,7)
                car.vertex_list[mutated_gene]=random_temp_car.vertex_list[mutated_gene]
        return car


    #the REAL crossover function
    #takes in two parents and returns a child
    def evolve2(self,parents):
        parents = [self.population_data[parents[0]].car_def, self.population_data[parents[1]].car_def]
        child = car_info()


        child.set_wheel_count(2) #set wheel count, fixed at 2

        for i in range(child.get_wheel_count()):
            if bool(random.getrandbits(1)):
                child.wheel_radius[i]=parents[0].wheel_radius[i]  #set wheel radius
            else:
                 child.wheel_radius[i]=parents[1].wheel_radius[i]

            if bool(random.getrandbits(1)):
                 child.wheel_vertex[i] = parents[0].wheel_vertex[i]  #set wheel vertex
            else:
                 child.wheel_vertex[i] = parents[1].wheel_vertex[i]

            if bool(random.getrandbits(1)):
                child.wheel_density[i] = parents[0].wheel_density[i] #set wheel density
            else:
                child.wheel_density[i] = parents[1].wheel_density[i]

        for i in range(len(child.get_vertex_list())):
            if bool(random.getrandbits(1)):
                 child.vertex_list[i] = parents[0].vertex_list[i]  #sets the body vertecies
            else:
                 child.vertex_list[i] = parents[1].vertex_list[i]

            child.set_wheel_density(parents[1].get_wheel_density())

        if bool(random.getrandbits(1)):
            child.set_chassis_density(parents[0].get_chassis_density()) #sets the chassis density
        else:
            child.set_chassis_density(parents[1].get_chassis_density())

        #mutate that sneak mah
        child2=self.mutate2(child)
        child_car = car(self.world, random=False, car_def=child2)
        return child_car

    # def random_chassis_maker(self, vertex_list):
    #     vertex_list.append(b2Vec2(random.random() * chassisMaxAxis + chassisMinAxis, 0))
    #     vertex_list.append(b2Vec2(random.random() * chassisMaxAxis + chassisMinAxis,
    #                               random.random() * chassisMaxAxis + chassisMinAxis))
    #     vertex_list.append(b2Vec2(0, random.random() * chassisMaxAxis + chassisMinAxis))
    #     vertex_list.append(b2Vec2(-random.random() * chassisMaxAxis - chassisMinAxis,
    #                               random.random() * chassisMaxAxis + chassisMinAxis))
    #     vertex_list.append(b2Vec2(-random.random() * chassisMaxAxis - chassisMinAxis, 0))
    #     vertex_list.append(b2Vec2(-random.random() * chassisMaxAxis - chassisMinAxis,
    #                               -random.random() * chassisMaxAxis - chassisMinAxis))
    #     vertex_list.append(b2Vec2(0, -random.random() * chassisMaxAxis - chassisMinAxis))
    #     vertex_list.append(b2Vec2(random.random() * chassisMaxAxis + chassisMinAxis,
    #                               -random.random() * chassisMaxAxis - chassisMinAxis))
    #     return vertex_list
    #
    # def chassis_check(self,chassis_list):
    #     new_chassis=[]
    #     first_row=chassis_list[0]
    #     if first_row[0]>chassisMaxDensity or first_row[0]<chassisMinAxis:  #row1
    #         return self.random_chassis_maker(new_chassis)
    #     if first_row[0] != 0:
    #         return self.random_chassis_maker(new_chassis)
    #
    #     second_row=chassis_list[1]
    #     if second_row[0]>chassisMaxDensity or second_row[0]<chassisMinAxis: #row2
    #         return self.random_chassis_maker(new_chassis)
    #     if second_row[1]>chassisMaxDensity or second_row[1]<chassisMinAxis:
    #         return self.random_chassis_maker(new_chassis)
    #
    #     third_row=chassis_list[2]
    #     if third_row[0] != 0:
    #         return self.random_chassis_maker(new_chassis)
    #     if third_row[1]>chassisMaxDensity or third_row[1]<chassisMinAxis:
    #         return self.random_chassis_maker(new_chassis)
    #
    #     fourth_row=chassis_list[3]
    #     if fourth_row[0] < (0-chassisMaxAxis) or fourth_row[0] > (0-chassisMinAxis):
    #         return self.random_chassis_maker(new_chassis)
    #     if fourth_row[1]>chassisMaxDensity or fourth_row[1]<chassisMinAxis:
    #         return self.random_chassis_maker(new_chassis)
    #
    #     fifth_row=chassis_list[4]
    #     if fifth_row[0] < (0-chassisMaxAxis) or fifth_row[0] > (0-chassisMinAxis):
    #         return self.random_chassis_maker(new_chassis)
    #     if fifth_row[1] !=0:
    #         return self.random_chassis_maker(new_chassis)
    #
    #     sixth_row=chassis_list[5]
    #     if sixth_row[0] < (0-chassisMaxAxis) or sixth_row[0] > (0-chassisMinAxis):
    #         return self.random_chassis_maker(new_chassis)
    #     if sixth_row[1] < (0-chassisMaxAxis) or sixth_row[1] > (0-chassisMinAxis):
    #         return self.random_chassis_maker(new_chassis)
    #
    #     seventh_row=chassis_list[6]
    #     if seventh_row[0] !=0:
    #         return self.random_chassis_maker(new_chassis)
    #     if seventh_row[1] < (0-chassisMaxAxis) or seventh_row[1] > (0-chassisMinAxis):
    #         return self.random_chassis_maker(new_chassis)
    #
    #     eigth_row=chassis_list[7]
    #     if eigth_row[0]>chassisMaxDensity or eigth_row[0]<chassisMinAxis:
    #         return self.random_chassis_maker(new_chassis)
    #     if eigth_row[1] < (0-chassisMaxAxis) or eigth_row[1] > (0-chassisMinAxis):
    #         return self.random_chassis_maker(new_chassis)
    #
    #     return chassis_list

    # #converts a binary car to normal car
    # def binary_to_car(self, binary_car):
    #     list_of_b=[]
    #
    #
    #     binary_car1=list(binary_car)
    #
    #     for x in range(len(binary_car)):
    #         if binary_car1[x]=='-' or binary_car1[x]=='b':
    #             binary_car1[x]='0'
    #
    #     binary_car1[1]='b'
    #     binary_car1[12]='b'
    #     binary_car1[23]='b'
    #     binary_car1[34]='b'
    #     binary_car1[45]='b'
    #     binary_car1[56]='b'
    #     binary_car1[67]='b'
    #     binary_car1[78]='b'
    #     binary_car1[89]='b'
    #     binary_car1[100]='b'
    #     binary_car1[111]='b'
    #     binary_car1[122]='b'
    #     binary_car1[133]='b'
    #     binary_car1[143]='-'
    #     binary_car1[145]='b'
    #     binary_car1[155]='b'
    #     binary_car1[165]='-'
    #     binary_car1[167]='b'
    #     binary_car1[177]='b'
    #     binary_car1[187]='-'
    #     binary_car1[198]='-'
    #     binary_car1[189]='b'
    #     binary_car1[200]='b'
    #     binary_car1[210]='b'
    #     binary_car1[220]='-'
    #     binary_car1[222]='b'
    #     binary_car1[232]='b'
    #     binary_car1[242]='-'
    #     binary_car1[244]='b'
    #     binary_car=''.join(binary_car1)
    #
    #     for x in range(len(binary_car)-1):
    #         if binary_car[x]=='-' and binary_car[x+1]== 'b':
    #             binary_car[x+1]=='0'
    #
    #
    #     new_car=car_info()
    #     wheel_radius_values=[int(binary_car[:11],2) / 100,int(binary_car[11:22],2) /100]
    #     wheel_density_values=[int(binary_car[22:33],2) / 10,int(binary_car[33:44],2) /10]
    #     wheel_vertex_list=[int(binary_car[44:55],2),int(binary_car[55:66],2)]
    #     chassisDensity=int(binary_car[66:77],2)
    #
    #     # print binary_car[143:154]
    #     # print binary_car[154:165]
    #     # print binary_car[165:176]
    #     # print binary_car[176:187]
    #     # print binary_car[187:198]
    #     # print binary_car[198:209]
    #     # print binary_car[209:220]
    #     # print binary_car[220:231]
    #     # print binary_car[231:242]
    #     # print binary_car[242:]
    #
    #     #print int(binary_car[77:88],2)
    #     #print float(int(binary_car[77:88],2))
    #     #print float(int(binary_car[77:88],2))/100
    #
    #     vertex_list1=[]
    #     vertex_list1.append(b2Vec2(float(int(binary_car[77:88],2)) / 100,float(int(binary_car[88:99],2)) /100)) #1
    #     vertex_list1.append(b2Vec2(float(int(binary_car[99:110],2)) / 100,float(int(binary_car[110:121],2)) /100)) #2
    #     vertex_list1.append(b2Vec2(float(int(binary_car[121:132],2)) / 100,float(int(binary_car[132:143],2)) /100)) #3
    #     vertex_list1.append(b2Vec2(float(int(binary_car[143:154],2)) / 100,float(int(binary_car[154:165],2)) /100)) #4
    #     vertex_list1.append(b2Vec2(float(int(binary_car[165:176],2)) / 100,float(int(binary_car[176:187],2)) /100)) #5
    #     vertex_list1.append(b2Vec2(float(int(binary_car[187:198],2)) / 100,float(int(binary_car[198:209],2)) /100))#6
    #     vertex_list1.append(b2Vec2(float(int(binary_car[209:220],2)) / 100,float(int(binary_car[220:231],2)) /100)) #7
    #     vertex_list1.append(b2Vec2(float(int(binary_car[231:242],2)) / 100,float(int(binary_car[242:],2)) /100)) #8
    #
    #     mutate=randint(0,100)
    #     if mutate<30:
    #         wheel_radius_values[0]*=3
    #         wheel_radius_values[1]*=3
    #
    #     if wheel_radius_values[0]>wheelMaxRadius:
    #         wheel_radius_values[0]=wheelMinRadius
    #     if wheel_radius_values[0]<wheelMinRadius:
    #         wheel_radius_values[0]=wheelMinRadius+.1
    #
    #     if wheel_radius_values[1]>wheelMaxRadius:
    #         wheel_radius_values[1]=wheelMaxRadius
    #     if wheel_radius_values[1]<wheelMinRadius:
    #         wheel_radius_values[1]=wheelMinRadius
    #
    #     if wheel_density_values[0]>wheelMaxDensity:
    #         wheel_density_values[0]=wheelMaxDensity
    #     if wheel_density_values[0]<wheelMinDensity:
    #         wheel_density_values[0]=wheelMinDensity
    #
    #     if wheel_density_values[1]>wheelMaxDensity:
    #         wheel_density_values[1]=wheelMaxDensity
    #     if wheel_density_values[1]<wheelMinDensity:
    #         wheel_density_values[1]=wheelMinDensity
    #
    #     if chassisDensity>chassisMaxDensity:
    #         chassisDensity=chassisMaxDensity
    #     if chassisDensity<chassisMinDensity:
    #         chassisDensity=chassisMinDensity
    #
    #     for x in range(8):
    #         temp=vertex_list1[x]
    #         if math.fabs(temp[0]) > chassisMaxAxis:
    #             if temp[0]<0:
    #                 temp[0]=0-chassisMaxAxis
    #             temp[0]=chassisMaxAxis
    #         if math.fabs(temp[0])< chassisMinAxis:
    #             if temp[0]<0:
    #                 temp[0]=0-chassisMinAxis
    #             temp[0]=chassisMinAxis
    #
    #         if math.fabs(temp[1]) > chassisMaxAxis:
    #             if temp[1]<0:
    #                 temp[1]=0-chassisMaxAxis
    #             temp[1]=chassisMaxAxis
    #         if math.fabs(temp[1])< chassisMinAxis:
    #             if temp[1]<0:
    #                 temp[1]=0-chassisMinAxis
    #             temp[1]=chassisMinAxis
    #
    #     temp=vertex_list1[0]
    #     temp[1]=0
    #
    #     temp=vertex_list1[2]
    #     temp[0]=0
    #
    #     temp=vertex_list1[4]
    #     temp[1]=0
    #
    #     temp=vertex_list1[7]
    #     temp[0]=0
    #
    #
    #     vertex_list1=self.chassis_check(vertex_list1)
    #
    #     new_car.set_wheel_count(2)
    #     new_car.set_wheel_density(wheel_density_values)
    #     new_car.set_wheel_radius(wheel_radius_values)
    #     new_car.set_wheel_vertex(wheel_vertex_list)
    #     new_car.set_chassis_density(chassisDensity)
    #     new_car.set_vertex_list(vertex_list1)
    #
    #     # print new_car.get_wheel_count()
    #     # print new_car.get_wheel_radius()
    #     # print new_car.get_wheel_vertex()
    #     # print new_car.get_chassis_density()
    #     # print new_car.get_vertex_list()
    #     # print vertex_list1
    #     # print new_car.get_wheel_density()
    #
    #
    #     brand_new_car = car(self.world, random=False, car_def=new_car)
    #
    #     #except AssertionError:
    #
    #     #     new_list=[]
    #     #     new_car.set_vertex_list(self.random_chassis_maker(new_list))
    #     #     brand_new_car=car(self.world, random=False, car_def=new_car)
    #     #     exit(1)
    #    return brand_new_car


    #actually creates the new generation
    def next_generation(self):
        global max_distance
        self.sort_distance2()
        n = 8 #keeps 8 from the previous generation
        new_population = []
        new_population_data = []
        #appends into the new population
        for i in range(n):
            new_car = car(self.world, random=False, car_def=self.population_data[i].car_def)
            new_population.append([new_car.chassis, new_car.wheels])
            new_population_data.append(car_data(self.population_data[i].chassis, self.population_data[i].wheels,
                                                self.population_data[i].car_def))


        mates_list = []  # pairs of indices of parents that have mated (to avoid duplicates in the same generation)
        while len(new_population) < (self.population_size)*.8:
            parents = self.get_parent_index(mates_list)
            mates_list.append(parents)
            child=self.evolve2(parents) #makes the kids

            new_population.append([child.chassis, child.wheels])
            new_population_data.append(car_data(child.chassis, child.wheels, child.car_def))

        # change generation.
        while (len(new_population) < (self.population_size)):
            old_car=self.make_random_car()
            new_car = car(self.world, random=False, car_def=old_car) #self.population_data[i].car_def)
            new_population.append([new_car.chassis,new_car.wheels])
            new_population_data.append((car_data(new_car.chassis,new_car.wheels,new_car.car_def)))


        self.killed = 0
        for index, elem in enumerate(new_population_data):
            self.population_data[index] = elem

        for index, elem in enumerate(new_population):
            self.population[index] = elem


    def create_generation_1(self):
        for i in range(self.population_size):
            temp = car(self.world)
            self.population.append([temp.get_car_chassis(), temp.get_car_wheels()])
            self.population_data.append(car_data(temp.get_car_chassis(), temp.get_car_wheels(), temp.car_def))

m = do_stuff()
