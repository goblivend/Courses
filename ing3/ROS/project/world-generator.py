import math
import matplotlib.pyplot as plt

COLOR_RED = '1 0 0 1'
COLOR_YELLOW = '1 1 0 1'

def generate_cone(pos, color, name) :
    (x, y) = pos
    return f'''
    <model name='{name}'>
       <pose>{x} {y} 0.5 0 -0 0</pose>
       <link name='cylinder_link'>
         <inertial>
           <inertia>
             <ixx>0.14580000000000001</ixx>
             <ixy>0</ixy>
             <ixz>0</ixz>
             <iyy>0.14580000000000001</iyy>
             <iyz>0</iyz>
             <izz>0.125</izz>
           </inertia>
           <mass>1</mass>
           <pose>0 0 0 0 -0 0</pose>
         </inertial>
         <collision name='cylinder_collision'>
           <geometry>
             <cylinder>
               <radius>0.5</radius>
               <length>1</length>
             </cylinder>
           </geometry>
           <surface>
             <friction>
               <ode/>
             </friction>
             <bounce/>
             <contact/>
           </surface>
         </collision>
         <visual name='cylinder_visual'>
           <geometry>
             <cylinder>
               <radius>0.5</radius>
               <length>1</length>
             </cylinder>
           </geometry>
           <material>
             <ambient>{color}</ambient>
             <diffuse>{color}</diffuse>
             <specular>1 1 1 1</specular>
           </material>
         </visual>
         <pose>0 0 0 0 -0 0</pose>
         <enable_wind>false</enable_wind>
       </link>
       <static>false</static>
       <self_collide>false</self_collide>
     </model>
'''

def generate_world_file(models) :
    models_str = "\n".join(models)
    return f'''<sdf version='1.9'>
  <world name='empty'>
    <physics name='1ms' type='ignored'>
      <max_step_size>0.02</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>100</real_time_update_rate>
    </physics>
    <plugin name='gz::sim::systems::Physics' filename='ignition-gazebo-physics-system'/>
    <plugin name='gz::sim::systems::UserCommands' filename='ignition-gazebo-user-commands-system'/>
    <plugin name='gz::sim::systems::SceneBroadcaster' filename='ignition-gazebo-scene-broadcaster-system'/>
    <plugin name='gz::sim::systems::Contact' filename='ignition-gazebo-contact-system'/>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type='adiabatic'/>
    <scene>
      <ambient>1 1 1 1</ambient>
      <background>1 1 1 1</background>
      <shadows>false</shadows>
    </scene>
    <model name='ground_plane'>
      <static>true</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>1000 1000</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode/>
            </friction>
            <bounce/>
            <contact/>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>1000 1000</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
        <pose>0 0 0 0 -0 0</pose>
        <inertial>
          <pose>0 0 0 0 -0 0</pose>
          <mass>1</mass>
          <inertia>
            <ixx>1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1</iyy>
            <iyz>0</iyz>
            <izz>1</izz>
          </inertia>
        </inertial>
        <enable_wind>false</enable_wind>
      </link>
      <pose>0 0 0 0 -0 0</pose>
      <self_collide>false</self_collide>
    </model>
    <light name='sun' type='directional'>
      <pose>0 0 10 0 -0 0</pose>
      <cast_shadows>true</cast_shadows>
      <intensity>1</intensity>
      <direction>-0.5 0.1 -0.9</direction>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <linear>0.01</linear>
        <constant>0.90000000000000002</constant>
        <quadratic>0.001</quadratic>
      </attenuation>
      <spot>
        <inner_angle>0</inner_angle>
        <outer_angle>0</outer_angle>
        <falloff>0</falloff>
      </spot>
    </light>



    {models_str}




  </world>
</sdf>
'''


class PathInstruction:
    def __init__(self, type) :
        self.type = type

class Line(PathInstruction):
    def __init__(self, length) :
        super().__init__("line")
        self.length = length

class ArcLeft(PathInstruction):
    def __init__(self, radius, angle) :
        super().__init__("arc_left")
        self.radius = radius
        self.angle = angle

class ArcRight(PathInstruction):
    def __init__(self, radius, angle) :
        super().__init__("arc_right")
        self.radius = radius
        self.angle = angle


def visualize_path(points):
    x_coords = [p[0] for p in points]
    y_coords = [p[1] for p in points]
    plt.plot(x_coords, y_coords, marker="o", linestyle="-")
    plt.xlabel("X-axis")
    plt.ylabel("Y-axis")
    plt.title("Generated Path")
    plt.axis("equal")  # Ensures the aspect ratio is correct
    plt.grid(True)
    plt.show()


def visualize_route(center_points, left_points, right_points):
    x_coords = [p[0] for p in center_points]
    y_coords = [p[1] for p in center_points]
    plt.plot(x_coords, y_coords, marker="o", linestyle="-", color="blue")
    x_coords = [p[0] for p in left_points]
    y_coords = [p[1] for p in left_points]
    plt.plot(x_coords, y_coords, marker="o", linestyle="-", color="red")
    x_coords = [p[0] for p in right_points]
    y_coords = [p[1] for p in right_points]
    plt.plot(x_coords, y_coords, marker="o", linestyle="-", color="yellow")
    plt.xlabel("X-axis")
    plt.ylabel("Y-axis")
    plt.title("Generated Path")
    plt.axis("equal")  # Ensures the aspect ratio is correct
    plt.grid(True)
    plt.show()




def normalise(vect) :
    norm = (vect[0]**2 + vect[1]**2)**0.5
    return [vect[0]/norm, vect[1]/norm]

def rotate_vector(vector, angle):
    [x, y] = vector
    new_x = x * math.cos(angle) - y * math.sin(angle)
    new_y = x * math.sin(angle) + y * math.cos(angle)
    return [new_x, new_y]

def generate_line(start, dir, length, plot_distance, remaining_dist) :
    line = []
    i = remaining_dist + plot_distance
    while i <= length :    #for i in range(remaining_dist, length+1, plot_distance) :
        line.append([start[0] - i*math.sin(dir), start[1] + i*math.cos(dir)])
        i += plot_distance
    print(line, length, i)
    return line, [start[0] - length*math.sin(dir), start[1] + length*math.cos(dir)], dir, i - length - plot_distance

def generate_arc_left(start, dir, radius, angle, plot_distance, remaining_dist) :
    arc = []
    circle_center = [start[0] - math.sin(dir+math.pi/2)*radius, start[1] +math.cos(dir+math.pi/2)*radius]
    # print(circle_center)
    new_dir = dir + angle

    i = remaining_dist + plot_distance
    while i <= angle * radius :
        arc.append([circle_center[0] + radius*math.cos(dir + i / radius), circle_center[1] + radius*math.sin(dir + i / radius)])
        i += plot_distance
    # print(list(range(int(0 * radius), int(angle * radius)+1, plot_distance)))
    return arc, [circle_center[0] + radius*math.cos(new_dir), circle_center[1] + radius*math.sin(new_dir)], new_dir, i - angle * radius - plot_distance


def generate_arc_right(start, dir, radius, angle, plot_distance, remaining_dist) :
    arc = []
    circle_center = [start[0] + math.sin(dir+math.pi/2)*radius, start[1] -math.cos(dir+math.pi/2)*radius]
    # print(circle_center)
    new_dir = dir - angle
    i = remaining_dist + plot_distance
    while i <= angle * radius : # for i in range(remaining_dist, int()+1, plot_distance) :
        arc.append([circle_center[0] + radius*math.cos(dir - math.pi - i / radius), circle_center[1] + radius*math.sin(dir - math.pi - i / radius)])
        i += plot_distance
    # print(list(range(int(0 * radius), int(angle * radius)+1, plot_distance)))
    return arc, [circle_center[0] + radius*math.cos(dir - math.pi - angle), circle_center[1] + radius*math.sin(dir - math.pi - angle)], new_dir, i - angle * radius - plot_distance

def generate_path(start, dir, instructions, plot_distance) :
    path = []
    end = start
    remaining_dist = -plot_distance # Distance since last plot (avoid having a plot at the end of an instruction and another one at the next beginning)
    for instruction in instructions :
        if instruction.type == "line" :
            tmp_path, end, dir, remaining_dist = generate_line(end, dir, instruction.length, plot_distance, remaining_dist)
            path += tmp_path
        elif instruction.type == "arc_left" :
            tmp_path, end, dir, remaining_dist = generate_arc_left(end, dir, instruction.radius, instruction.angle, plot_distance, remaining_dist)
            path += tmp_path
        elif instruction.type == "arc_right" :
            tmp_path, end, dir, remaining_dist = generate_arc_right(end, dir, instruction.radius, instruction.angle, plot_distance, remaining_dist)
            path += tmp_path
        print(remaining_dist)
    # print(path)
    return path

def generate_offset_instructions(instructions, offset) :
    def fix_instruction(instruction) :
        if instruction.type == "arc_left" :
            return ArcLeft(instruction.radius + offset, instruction.angle)
        elif instruction.type == "arc_right" :
            return ArcRight(instruction.radius - offset, instruction.angle)
        return instruction
    res = map(fix_instruction, instructions)
    return res

def generate_complete_path(route, dir=-math.pi/2, sides_distance=4, plot_distance=3) :
    base_route = generate_path([0, 0], dir, route, plot_distance)

    start_red = rotate_vector([-sides_distance, 0], dir)
    start_yellow = rotate_vector([sides_distance, 0], dir)

    red_route = generate_path(start_red, dir, generate_offset_instructions(route, -sides_distance), plot_distance)
    yellow_route = generate_path(start_yellow, dir, generate_offset_instructions(route, sides_distance), plot_distance)

    return base_route, red_route, yellow_route

my_route = [
    Line(15),
    ArcLeft(15, math.pi/2),
    Line(15),
    ArcRight(15, math.pi/2),
    Line(15),
    ArcRight(15, math.pi/2),
    Line(45),
    ArcRight(15, math.pi/2),
    Line(60),
    ArcRight(15, math.pi),
]

base, red, yellow = generate_complete_path(my_route)
visualize_route(base, red, yellow)


# models = []
# for i, point in enumerate(red) :
#     models.append(generate_cone(point, COLOR_RED, f"red_cone_{i}"))

# for i, point in enumerate(yellow) :
#     models.append(generate_cone(point, COLOR_YELLOW, f"yellow_cone_{i}"))


# with open("generated_world.sdf", "w") as f :
#     f.write(generate_world_file(models))
