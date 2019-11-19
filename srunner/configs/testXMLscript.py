
from xml.etree.ElementTree import Element, SubElement, Comment, tostring
import xml.etree.ElementTree as ET
from xml.dom import minidom


def prettify(elem):
    """Return a pretty-printed XML string for the Element.
    """
    rough_string = ET.tostring(elem, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")


class vehicle(object):
    # Initializer / Instance Attributes
    def __init__(self, x_i=0, x_f=2, x_step=1,
                 y_i=0, y_f=2, y_step=1, yaw_i=0, yaw_f=2, yaw_step=1, model=''):
        self.x_set = range(x_i, x_f, x_step)
        self.y_set = range(y_i, y_f, y_step)
        self.yaw_set = range(yaw_i, yaw_f, yaw_step)
        self.param_set = []
        self.model = str(model)
        for x in self.x_set:
            for y in self.y_set:
                for yaw in self.yaw_set:
                    param = {'x': str(x), 'y': str(y), 'yaw': str(yaw), 'model': str(model)}
                    self.param_set.append(param)


model = 'vehicle.tesla.model3' #param
name = 'ConfrontationCross_'  # param
scenario_type = "ConfrontationCross"  # param
town = 'Town2'

vehicle_param_set = []
# car parameters
vehicle1 = vehicle(x_i=0, x_f=2, x_step=1, y_i=0, y_f=2, y_step=1, yaw_i=0, yaw_f=2, yaw_step=1, model=model)
vehicle2 = vehicle(x_i=3, x_f=5, x_step=1, y_i=-1, y_f=2, y_step=1, yaw_i=0, yaw_f=2, yaw_step=1, model=model)
vehicle3 = vehicle(x_i=6, x_f=7, x_step=1, y_i=-1, y_f=2, y_step=1, yaw_i=0, yaw_f=2, yaw_step=1, model=model)

vehicle_param_set.append(vehicle1.param_set)
vehicle_param_set.append(vehicle2.param_set)
vehicle_param_set.append(vehicle3.param_set)

num_vehicle = len(vehicle_param_set)
var = [0]*num_vehicle

scenarios = Element('scenarios')

i = 0
stop = True
while stop:
    scenario = SubElement(scenarios, 'scenario',
                          attrib={'name': name+str(i), 'type': scenario_type,
                                  'town': town})
    temp = var[0]
    vehicle = SubElement(scenario, 'ego_vehicle', attrib=vehicle_param_set[0][var[0]])
    for n in range(1, num_vehicle):
        vehicle = SubElement(scenario, 'other_actor', attrib=vehicle_param_set[n][var[n]])
    i += 1
    var[num_vehicle-1] += 1
    for n in range(num_vehicle-1, -1, -1):

        if var[n] == len(vehicle_param_set[n]):

            if n == 0:
                stop = False

            var[n] = 0
            var[n - 1] += 1

mydata = prettify(scenarios)
myfile = open("{}}.xml".format(scenario_type), "w")
myfile.write(mydata)

# for vehicle_param in vehicle_param_set:
#     for
# scenario = SubElement(scenarios, 'scenario',
#                       attrib={'name': name+str(i), 'type': "ConfrontationCross",
#                               'town': 'Town2'})
#
# for x in x_set:
#     for y in y_set:
#         for yaw in yaw_set:
#
#             if i == 0:
#                 actor = 'ego_vehicle'
#             else:
#                 actor = 'other_actor'
#             i += 1
#
#             vehicle = SubElement(scenario, actor, attrib={'x': str(x), 'y': str(y), 'yaw': str(yaw), 'model': str(model)})
#
# # scenario.text = 'This child contains text.'
# # child_with_tail = SubElement(scenarios, 'child_with_tail')
# # child_with_tail.text = 'This child has regular text.'
# # child_with_tail.tail = 'And "tail" text.'

