-Download the repository from https://github.com/XHao1997/InverseKinematics_Sim/archive/refs/heads/master.zip
-Extract the folder on the Desktop (or in the preferred directory)
-Download Anaconda from this link
-Install Anaconda
-Open Anaconda Prompt (Anaconda3) from the Start menu
-In the terminal use cd and the directory where did you extracted the folder(for example cd Desktop/FOLDERNAME)
-Type conda env create -f robot.yml
-Type conda activate robotarm
-Type conda install pywin32=227
-Type python -m ipykernel install --user --name=python3
-Type jupyter notebook  
-You are ready to use jupyter. 

-Every time that we want to start to work again we have to  

-Open Anaconda Prompt (Anaconda3) in Start 
-Go to the directory of folder (for example cd Desktop/FOLDERNAME)
-Type conda activate robotarm (this will activate the environment robotarm)
-Type jupyter notebook (this will open the notebook with all the files) 