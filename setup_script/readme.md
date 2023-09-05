## Script to setup Aerialist from scratch. 
We recommend you to use a fresh installed Ubuntu environment to set up Aerialist in order to avoid dependency issues. 
The script was tested in a virtual machine in VMWare. 
#### You can use VMWare Workstation Player(free version) to setup the clean Ubuntu environment..


Copy the script into your system's home directory and make it executable. To make it executable, open the terminal and navigate to file's location and type below command:

```
chmod +x full_setup.sh
```

### Now you can run the executable script by typing below command :

```
./full_setup.sh 
```

#### You will be prompted to enter the root password. 

After that the installation will continue for at least half an hour to around one hour depending on your Internet speed.

Once the script has completed successfully, you can close the terminal and open the new terminal.
In the new terminal, you can see the conda base environment has been now activate.

#### You can switch to Aerialist environment by typing below command:

```
conda activate Aerialist.
```

Now navigate into Aerialist directory which is located in your home directory by entering below command in the terminal.

```
cd ~/Aerialist
```

#### Now you are ready to use and test Aerialist. 