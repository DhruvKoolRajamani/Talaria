# Talaria

Install [platformio](https://platformio.org/platformio-ide) for any IDE/non-IDE.  

**VSCODE**: The following extensions will also be helpful:
 - [CPP Extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools)
 - [Doxygen Documentation](https://marketplace.visualstudio.com/items?itemName=cschlosser.doxdocgen)
 - [GitLens](https://marketplace.visualstudio.com/items?itemName=eamodio.gitlens)
 - [Platformio](https://marketplace.visualstudio.com/items?itemName=platformio.platformio-ide)

## Formatting

**PLEASE INSTALL clang-format if you're making any changes to the code**  

```bash
sudo apt install clang-format
```

Windows:  
 - [x64](http://llvm.org/releases/3.7.0/LLVM-3.7.0-win64.exe)
 - [x32](http://llvm.org/releases/3.7.0/LLVM-3.7.0-win32.exe)

## Initializing your workspace

Open your terminal and follow the steps below  

```bash
# cd to your workspace, in my case it's home
# I'll use my own workspace, but you can replace the directory
cd ~/
git clone https://github.com/DhruvKoolRajamani/Talaria.git
cd ~/Talaria # cd to your controllers workspace.
pio init --ide vscode # Change vscode to whatever ide you use eg. atom
pio run -e Talaria-main
```

## Uploading

```bash
cd ~/Talaria
pio run -e <env_name> -t upload
```

## Running with ROS Serial

```bash
# Open a new terminal
roscore

# Open a new terminal
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0

# Open a new terminal
rostopic echo /network_strings
```

## Creating New Ros Messages

# For New package 
    # in catkin_ws
    cd src
    catkin_create_pkg [Name] <dependencies>
    cd [package]
    mkdir msg
# For new message in existing package (cd into package/msg dir)
    #here is where the message .msg file goes
    # update package.xml and CmakeLists in similar fashion to Ros tutorials and previously created messages
    
    cd ../.. # into catkin_ws
    catkin_make
    source ~/.bashrc #assuming source ~/../catkin_ws/devel/setup.bash in .bashrc
    cd ../lib
    rm -rf ros_lib
    rosrun rosserial_mbed make_libraries.py .
