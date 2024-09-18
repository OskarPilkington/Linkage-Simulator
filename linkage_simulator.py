import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation


class LinkageSimulator:
    # This is a class to define a mechanical linkage system. 
    # All modifications should be done through API calls to ensure everything is referenced correctly
    def __init__(self):
        # Dictionary to hold joints
        self.joints = {}
        self.current_id = 0
        self.linkage_list = []


    def new_joint(self, type="passive", coordinates=None, motor_linkage=None, motor_parent=None):
        
        if type == "passive":
            if coordinates and not is_tuple_of_two_numbers(coordinates):
                raise ValueError('coordinates must be a tuple with 2 numbers')
            joint = Joint(type=type, coordinates=coordinates)

        elif type == "static":
            if not is_tuple_of_two_numbers(coordinates):
                raise ValueError('coordinates must be a tuple with 2 numbers')
            joint = Joint(type=type, coordinates=coordinates)

        elif type == "motor":
            joint = Joint(type=type, motor_linkage=motor_linkage, motor_parent=motor_parent)
        else:
            raise ValueError('type must be either "passive", "static" or "motor"')
        
        index = self.current_id
        self.current_id += 1
        self.joints.update({index: joint})

        if type == "motor":
            self.add_link(index, motor_parent, motor_linkage)

        return index
    
    def add_link(self, joint_1, joint_2, length):
        # Adds a link between the nodes representing the specified joints. These are then used to calculate the closed form solution
        j1 = self.joints[joint_1]
        j2 = self.joints[joint_2]

        j1.linkages.append((joint_2, length))
        j2.linkages.append((joint_1, length))

        self.linkage_list.append((joint_1, joint_2))
    
    def print_joints(self):
        print(self.joints)

    def display(self):
        self.fig, self.ax = plt.subplots()
        self.update_fig()
        plt.show()

    def update_fig(self):
        # Plot static joints
        x_values = []
        y_values = []
        indexes = []
        
        for (id, joint) in self.joints.items():
            if joint.type == "static":
                (x, y) = joint.coordinates
                x_values.append(x)
                y_values.append(y)
                indexes.append(id)
                self.ax.text(x, y, str(id), fontsize=12, ha='right', va='bottom')
        
        self.ax.scatter(x_values, y_values,zorder=2)
        
        # Plot passive joints
        x_values = []
        y_values = []
        indexes = []
        
        for (id, joint) in self.joints.items():
            if joint.type == "passive":
                (x, y) = joint.coordinates
                x_values.append(x)
                y_values.append(y)
                indexes.append(id)
                self.ax.text(x, y, str(id), fontsize=12, ha='right', va='bottom')
        
        self.ax.scatter(x_values, y_values, zorder=2)

        # Plot motors
        x_values = []
        y_values = []
        indexes = []
        
        for (id, joint) in self.joints.items():
            if joint.type == "motor":
                (x, y) = joint.coordinates
                x_values.append(x)
                y_values.append(y)
                indexes.append(id)
                self.ax.text(x, y, str(id), fontsize=12, ha='right', va='bottom')
        
        self.ax.scatter(x_values, y_values, zorder=2)
        
        # Plot linkages
        for (j1_id, j2_id) in self.linkage_list:
            j1 = self.joints[j1_id]
            j2 = self.joints[j2_id]
            x_values = [j1.coordinates[0], j2.coordinates[0]]
            y_values = [j1.coordinates[1], j2.coordinates[1]]
            
            self.ax.plot(x_values, y_values, color="grey",zorder=1)

        self.ax.set_aspect('equal')

    def update_animation(self, frame):
        self.calculate_closed_form_kinematics(motor_angle=frame*2*math.pi/100)
        self.ax.clear()
        self.update_fig()
        self.ax.set_ylim(0,100)
        self.ax.set_xlim(0,90)
    
    def animate(self):
        self.fig, self.ax = plt.subplots()
        self.update_fig()

        ani = animation.FuncAnimation(self.fig, self.update_animation, frames=100)
        
        plt.show()

    def calculate_closed_form_kinematics(self, motor_angle=0):
        # Start off by setting the defined state of all passive nodes to False.
        defined_joint_cnt = 0

        for j in self.joints.values():
            if j.type == "passive":
                j.defined = False
            
            elif j.type == "static":
                j.defined = True
                defined_joint_cnt += 1
            
            elif j.type == "motor":
                anker = self.joints[j.motor_parent]
                motor_x = anker.coordinates[0] + j.motor_linkage*math.cos(motor_angle)
                motor_y = anker.coordinates[1] + j.motor_linkage*math.sin(motor_angle)
                j.coordinates = (motor_x, motor_y)
                j.defined = True
                defined_joint_cnt += 1
        
        while defined_joint_cnt < len(self.joints):
            # Go through all joints, and for those that have 2 defined neighbours, calculate position
            for (id, joint) in self.joints.items():
                # Do not check if joint location is defined
                if joint.defined:
                    continue
                
                # Check how many neighbors are defined
                defined_neighbours = []
                for (neighbour_id, length)  in joint.linkages:
                    if self.joints[neighbour_id].defined:
                        defined_neighbours.append((neighbour_id, length))
                
                if len(defined_neighbours) < 2:
                    continue
                elif len(defined_neighbours) > 2:
                    raise ValueError("Structure is overconstrained and thus invalid")
                else:
                    # Go from IDs to the objects
                    j1 = self.joints[defined_neighbours[0][0]]
                    j2 = self.joints[defined_neighbours[1][0]]

                    # Get the linkage lengths
                    r1 = defined_neighbours[0][1]
                    r2 = defined_neighbours[1][1]
                    
                    # Extract coordinates
                    (x1, y1) = j1.coordinates
                    (x2, y2) = j2.coordinates

                    intersection_points = circle_intersection(x1, y1, r1, x2, y2, r2)
                    if not intersection_points:
                        raise ValueError("Linkage Structure does not work.")
                    # print(intersection_points)

                    # Check if the joint has coordinates defined if so pick the intersection point closer to the coordinate
                    if joint.coordinates is not None:
                        # Calculate distance between coordinate estimate and intersection points
                        d1 = (joint.coordinates[0]-intersection_points[0][0])**2 + (joint.coordinates[1]-intersection_points[0][1])**2
                        d2 = (joint.coordinates[0]-intersection_points[1][0])**2 + (joint.coordinates[1]-intersection_points[1][1])**2

                        if d1 < d2:
                            defined_coordinates = intersection_points[0]
                        else:
                            defined_coordinates = intersection_points[1]
                    else:
                        # If we do not have coordinate estimates, just pick the one above.
                        defined_coordinates = intersection_points[1]
                    
                    joint.coordinates = defined_coordinates
                    joint.defined = True
                    defined_joint_cnt += 1

            

class Joint:
    # This class defines a joint in a Linkage Simulator.
    # Parameters:
    #   Type:           either "passive", "static" or "motor"
    #   coordinates:    Forces position for "static" type joints. Suggests position for "passive" type joints.
    #   motor_linkage:  Only used for "motor" type joints. Defines the length of the linkage from the motors parent to the motor joint.
    #   motor_parent:   Only used for "motor" type joints.Defines the ID of the joint which the motor joint rotates around.
    def __init__(self, type="passive", coordinates=None, motor_linkage=None, motor_parent=None):
        self.type = type
        self.coordinates = coordinates
        self.motor_linkage = motor_linkage
        self.motor_parent = motor_parent
        self.linkages = []
        self.defined = False

def is_tuple_of_two_numbers(variable):
    # Helper function to check if a variable is a tuple with 2 numbers
    if isinstance(variable, tuple) and len(variable) == 2:
        return all(isinstance(i, (int, float)) for i in variable)
    return False

def circle_intersection(x1, y1, r1, x2, y2, r2):
    # Calculate the distance between the centers
    d = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    # Check if there is an intersection
    if d > r1 + r2:  # No intersection: the circles are too far apart
        return None
    if d < abs(r1 - r2):  # No intersection: one circle is inside the other
        return None
    if d == 0 and r1 == r2:  # Infinite number of intersection points: circles are identical
        return None

    # Find the point where the line through the circle intersection points crosses the line between the circle centers
    a = (r1**2 - r2**2 + d**2) / (2 * d)
    h = math.sqrt(r1**2 - a**2)

    # Point P2 is the point where the line through the intersection points crosses the line between the centers
    x3 = x1 + a * (x2 - x1) / d
    y3 = y1 + a * (y2 - y1) / d

    # Calculate the offset of the intersection points from point P2
    x4_1 = x3 + h * (y2 - y1) / d
    y4_1 = y3 - h * (x2 - x1) / d

    x4_2 = x3 - h * (y2 - y1) / d
    y4_2 = y3 + h * (x2 - x1) / d

    return (x4_1, y4_1), (x4_2, y4_2)

def norm_2(x,y):
    return math.sqrt(x**2 + y**2)


if __name__ == "__main__":
    LS = LinkageSimulator()
    # Defining joints
    anchor_left = LS.new_joint("static", coordinates=(50,70))
    anchor_right = LS.new_joint("static", coordinates=(70+10,60-10))
    motor = LS.new_joint("motor", motor_linkage=10, motor_parent=anchor_right)
    passive_1 = LS.new_joint("passive", coordinates=(30,100))
    passive_2 = LS.new_joint("passive", coordinates=(10,70))
    passive_3 = LS.new_joint("passive", coordinates=(20,20))
    passive_4 = LS.new_joint("passive", coordinates=(40,20))
    passive_5 = LS.new_joint("passive", coordinates=(30,0))

    # passive_1 = LS.new_joint("passive")
    # passive_2 = LS.new_joint("passive")
    # passive_3 = LS.new_joint("passive")
    # passive_4 = LS.new_joint("passive")
    # passive_5 = LS.new_joint("passive")
    # Defining connections
    LS.add_link(anchor_left, passive_1, 30)
    LS.add_link(motor, passive_1, norm_2(40,40)-10)
    LS.add_link(anchor_left, passive_2, 20)
    LS.add_link(passive_2, passive_1, norm_2(20,30))
    LS.add_link(passive_3, passive_2, norm_2(50,10))
    LS.add_link(anchor_left, passive_4, norm_2(50,10))
    LS.add_link(passive_3, passive_4, 20)
    LS.add_link(passive_3, passive_5, norm_2(20,10))
    LS.add_link(passive_4, passive_5, norm_2(20,10))
    LS.add_link(passive_4, motor, norm_2(30,40))

    

    print(f"anchor_left index: {anchor_left}, anchor_right index: {anchor_right}")
    
    LS.calculate_closed_form_kinematics(motor_angle=3*math.pi/2)

    # LS.display()
    LS.animate()