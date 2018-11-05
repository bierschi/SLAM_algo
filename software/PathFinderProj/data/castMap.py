mapFile  = open("hector_slam_test_p2_format (copy).pgm", "r")
formatedFile  = open("hector_slam_test_p2_format_db.pgm", "w")

completeFile = ""
i = 0
spaceCounter = 0

for line in mapFile:
    if(i > 2):
        line = line.replace("\n", "")
        completeFile = completeFile + line
    i=i+1

values = completeFile.split(" ")

completeFile = ""

for i in range(1, len(values)):
    completeFile = completeFile + values[i]
    completeFile = completeFile + " "
    if(i%400 == 0):
        completeFile = completeFile + "\n"



mapFile.close()
formatedFile.write(completeFile)
formatedFile.close()
