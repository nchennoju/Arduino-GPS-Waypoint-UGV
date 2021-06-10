from tkinter import filedialog
from tkinter import *

GPS_PROGRAM_PATH = 'C:\\Users\\nchen\\Documents\\PlatformIO\\Projects\\GPS_i2c_Slave\\src\\main.cpp' #'C:\\Users\\nchen\\Documents\\PlatformIO\\Projects\\GPS_Waypoint\\src\\main.cpp'

class FileParse:

    root = Tk()

    def __init__(self):
        self.root.filename = filedialog.askopenfilename(initialdir="C:\\Users\\nchen\\Desktop",
                                                        title="Select data file")
        #print("FILE Location: " + self.root.filename)
        self.fN = self.root.filename
        self.file = open(self.fN)

        raw = self.file.read()
        arrTmp = raw.split('\n')
        arr = []
        self.coordinates = []
        for line in arrTmp:
            if("<missionitem no" in line):
                arr.append(line.replace('\t', ''))

        for line in arr:
            if("WAYPOINT" in line):
                lonIndex, latIndex = line.index("lon")+5, line.index("lat")+5
                lonIndexEnd, latIndexEnd = lonIndex, latIndex
                while (line[lonIndexEnd] != '\"'):
                    lonIndexEnd += 1
                while (line[latIndexEnd] != '\"'):
                    latIndexEnd += 1
                self.coordinates.append([float(line[latIndex:latIndexEnd]), float(line[lonIndex:lonIndexEnd])])


    def arrFormatted(self):
        string = "const double coordinates[][2] = {"
        for i in range(len(self.coordinates) - 1):
            string += "{" + str(self.coordinates[i][0]) + ", " + str(self.coordinates[i][1]) + "}, "
        string += "{" + str(self.coordinates[len(self.coordinates) - 1][0]) + ", " + str(
            self.coordinates[len(self.coordinates) - 1][1]) + "}};\n"
        return string

    def updateArduinoFile(self):
        file = open(GPS_PROGRAM_PATH)
        lines = file.readlines()
        index = -1
        error_flag = False
        for i in range(len(lines)):
            if("const double coordinates[][2]" in lines[i]):
                index = i
        if(index == -1):
            for i in range(len(lines)):
                if ("// ----- GPS" in lines[i]):
                    index = i
            if(index == -1):
                print("ERROR: choose the file defined in path variable is incorrect")
                error_flag = True
            else:
                lines.insert(index+1, self.arrFormatted())
        else:
            lines[index] = self.arrFormatted()
        print(index)
        file.close()

        if(not error_flag):
            f = open(GPS_PROGRAM_PATH, 'w')
            f.writelines(lines)
            f.close()

f = FileParse()
print(f.arrFormatted())
f.updateArduinoFile()

print("\n\nPress ENTER to close...")
input()
