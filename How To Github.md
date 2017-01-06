#How to Github

[Github](https://github.com) is a very powerful tool used to collaborate on code. Now you can just use the desktop version of github, but today we'll learn the command prompt way

#Getting Started
You will need Git for Windows [Click here to go to the installer for windows](https://git-scm.com/download/win)

Think of Github as a remote storage space, like Google Drive for code. Except in this case, we will be using Git Bash to control the flow of work. To start, search and open Git Bash on you computer.

Now go to Github.com and sign in to the micdsrobotics git repository page. Click on "Clone or Download" and select HTTPS and copy the link.

Open your Git Bash Command Window, and navigate to the desired parent directory (like c:/users/person/desktop/github and type:

```
git clone https://github.com/blabel3/MicdsRobotics.git
``

That first part 'git' tells the bash that we want to use the git and clone makes a copy of the desired repository

#Next Steps
Now open up the file and make a change.

Right click the File Explorer and click 'open git bash here'

Type:
```
git status
```
This command will tell you what you changed.

#'Publishing' these changes
Now, to send these changes to the repository, there are three phases:
*Adding files to comitt
	*git add {file names}
*Making the commit locally
	*git commit -m "{insert commit comment here}"
*Push local commit to remote repository
	*git push