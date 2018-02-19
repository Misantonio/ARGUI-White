import xml.etree.ElementTree
from os.path import expanduser

class XMLParser(object):
    def __init__(self, file):
        self.e = xml.etree.ElementTree.parse(file).getroot()
        self.b = xml.etree.ElementTree.parse(file).getiterator('node')

    def get_num_drones(self):
        num_drones = 0
        for _ in self.e.findall('group'):
            num_drones += 1
        return num_drones

    def drones_start_position(self):
        xx = []
        yy = []
        for atype in self.b:
            h = atype.get('args')
            if h != None:
                x_index = h.find('x')
                x = h[x_index:].split(' ')[1]
                y_index = h.find('y')
                y = h[y_index:].split(' ')[1]
                xx.append(x)
                yy.append(y)
        return tuple(xx), tuple(yy)

    def drones_names(self):
        names = []
        for atype in self.b:
            h = atype.get('args')
            if h != None:
                name_index = h.find('model')
                name = h[name_index:].split(' ')[1]
                names.append(name)
        return names

if __name__ == '__main__':
    home_path = expanduser("~")
    pkg_path = '/tum_simulator_ws/src/tum_simulator/cvg_sim_gazebo/launch/'
    file_name = 'ardrone_multi.launch'

    xml_file = home_path + pkg_path + file_name

    parser = XMLParser(xml_file)
    print parser.drones_start_position()
    print parser.drones_names()