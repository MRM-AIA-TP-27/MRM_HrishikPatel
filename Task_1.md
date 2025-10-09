Task 1: Hello World Program
This task involved setting up a new local repository, creating a specific directory structure, and pushing an initial program file to the remote GitHub repository using the Linux command line.

Git Commands Used:
mkdir GitHub_task: Created the local directory for the project.

cd GitHub_task: Navigated into the new project directory.

echo 'print("Hello World!")' > hello.py: Created the sample program file.

git init -b main: Initialized a new local Git repository with the primary branch named main.

git remote add origin git@github.com:MRM-AIA-TP-27/MRM_HrishikPatel.git: Established the connection between the local repository and the remote GitHub repository using the secure SSH protocol.

git add .: Staged all new and modified files in the current directory for the next commit.

git commit -m "Task 1: Initial commit with Hello World program in Python": Created a permanent snapshot (commit) of the staged changes with a descriptive message.

git push -u origin main: Pushed the local main branch and its commits to the remote repository for the first time, setting up tracking (-u).
