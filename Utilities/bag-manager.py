import os
import subprocess
import time
import sys

#global variables
# hostname = "swarm-data-server"
# user = "swarm-bag-server"
# directoryOnSever = "/home/swarm-bag-server/swarm-bags/agent0"
# pathToLocalBags = "./rosbags"
# agent = "0"

hostname = sys.argv[1]
user = sys.argv[2]
directoryOnSever = sys.argv[3]
pathToLocalBags = sys.argv[4]

timeout = 5

def listCompareLocalOnServer(list1, list2):
    """
    Returns the items that are on list 1, but not on list 2
    """
    diff = list(set(list1) - set(list2))
    return diff

def listCompareBoth(list1, list2):
    list1_set = set(list1)
    list2_set = set(list2)

    if (list1_set & list2_set):
        return list1_set & list2_set
    else:
        return []

def checkFiles(remoteDirecotryIP, pathToRemoteDirectory, pathToLocalDirectory, compareFunction):
    """
    This function returns a list that represents files
    that are in a local directory but not in a 
    remote directory
    """

    #get a list of the files on the server
    p = subprocess.run(["ssh", remoteDirecotryIP, "ls", pathToRemoteDirectory], capture_output=True, text=True)
    if (p.returncode == 0):
        filesOnServer = p.stdout.split()
        #print(filesOnServer)

    else:
        print("Server ls failed, exiting ...")
        return

    #get a list of the files locally stored
    q = subprocess.run(["ls", pathToLocalDirectory], capture_output=True, text=True)
    if (q.returncode == 0):
        filesOnLocal = q.stdout.split()
        #print(filesOnLocal)
    else:
        #ls did not work
        print('Local ls failed, exiting ...')
        return


    return compareFunction(filesOnLocal,filesOnServer)

def cleanup():
    """
    This function checks the local and remote directory and
    deletes any local files that are already on the server
    """
    filesInBoth = checkFiles((user + '@' + hostname), directoryOnSever, pathToLocalBags, listCompareBoth)
    print("=========== DELETING LOCALY ===========")
    print(filesInBoth)
    print("=======================================")

    if filesInBoth:
        for file in filesInBoth:
            g = subprocess.run(["rm", "./rosbags/" + file])

def checkForServerDirectory(remoteDirecotryIP,pathToRemoteDirectory):
    print("Checking for remote directory ...")
    p = subprocess.run(["ssh", remoteDirecotryIP, "ls", pathToRemoteDirectory], capture_output=True, text=True)
    
    if (p.returncode == 0):
        print('Remote directory found')
        return 0
    else:
        print('No remote direcctory found, ')
        m = subprocess.run(["ssh", remoteDirecotryIP, "mkdir", directoryOnSever ] , capture_output=True, text=True)
        if (m.returncode == 0):
            print("remote directory successfuly created")
            return 0
        else:
            print('failed to create remote directory')
            return 1


#Check if the server can be reached on the current network
response = os.system("ping -c 1 " + hostname)

#If we didn't get a good ping right away, lets try to get it a couple more times
while (response != 0 and timeout > 0):
    response = os.system("ping -c 1 " + hostname)
    timeout -= 1
    time.sleep(1)

#Check the ping response
if response == 0:
    #the server is reachable
    print (hostname, 'is reachable!')

    directoryFound = checkForServerDirectory((user + '@' + hostname), directoryOnSever)

    if (directoryFound == 0):
        #Check if there are any local files that are not already on the server
        differentFilesOnLocal = checkFiles((user + '@' + hostname), directoryOnSever, pathToLocalBags, listCompareLocalOnServer)

        #If the server is up to date, run a cleanup
        if not differentFilesOnLocal:
            print("Server is up to date!")
            cleanup()

        #If the server is not up to date, copy over the new files, then run a cleanup
        else: 
            for file in differentFilesOnLocal:
                #print(file)
                destination = ((user + '@' + hostname) + ":" + directoryOnSever)
                p = subprocess.run(["scp", "./rosbags/" + file, destination])

            cleanup()

    else:
        print('Unable to find or create a remote directory')

else:
    #The server is not reachable,
    print (hostname, 'is unreachable')