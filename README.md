# RocketDriver_V3_0
Refactor of RocketDriver Embedded System V2
Originally written by [Dan Morgan](https://github.com/dcm628), with help from [Ivan Krat](https://github.com/cooligrek) and [Brandon Summers](https://github.com/BrandonFromHR).
Refactor by [Joe Kessler](https://github.com/Copperbotte).

## Description
This is the driver for a custom made microcontroller called an ALARA.  Multiple ALARAs are used throughout BLT and adjacent clubs as a specialized microcontroller.

# Installation Tutorial
Written by Joe Kessler, 2023 October 5.

This is likely your first impression of Git and Github, so this will be an end-to-end tutorial for how-to guide to help you install all the tools you need to build this driver on your computer locally, and basic usage of Git and Github.

## Installation
This repo is hosted on an online hosting service called Github.  Git is one of the most important tools you'll ever learn to use when collaboratively programming with a team of any size.  It's very rare that you'll find a company employing software engineers that dont use git.  

### What is Git?? What about Github???
Git is a "Version control system" that manages multiple users editing a codebase simultaneously by putting each user's edits onto discrete "commits" onto a timeline-like "branch."  Usually, a programmer will make a new branch from an existing branch, commit changes to the new branch, then submit a "pull request" to merge the branch back to the original.  If the pull request is accepted, the branch is "merged" and those commits become part of the history of the original branch.  

Git is interfaced using various clients such as Git Bash, Git GUI, an IDE plugin, or a Github client.  My experience lies in Git Bash on Windows, but Git for most Linux distros is extremely easy to install.

Github is an online git repository hosting service owned by Microsoft.  There are alternatives, but Github is the most popular at the moment. (2023 October 3)  You'll need an account on Github to "Push" commits to the repo, but you shouldn't need one to "Pull" from the repo. (I have not tested this)

If you're reading this you're probably going to be writing software so I cannot stress enough how important it is to get a Github account before you pull this repo.  Programmers usually use their Github accounts as portfolios, so having a Github account filled with quality code commits is an excellent way to prove your ability to prospective employers.  Senior engineers will look through your code, so make it impressive!

### How to pull the repo
I'm going to use Git Bash on Windows Terminal as an example.  Almost every Git client has the same interface, so the skills should transfer.  (You may need to login to the client to pull the repo, but I can't remember.  If this tutorial works, please update this file with the result!)
1. First, install a Git client on your computer.
2. Navigate to the Github repo and click on the green box that says <> Code ▾.
3. Copy the HTTPS link presented within the dropdown menu, or click on the ⮺ (overlapping square button) to copy the link.
4. Open a command line or terminal, and navigate to the folder that you want to install the repo into using the commands `cd` and `ls`.
5. Type in the following command with the completed link:
```bash
> git clone <https://github.com/... .git>
```

The repo should now be downloading to your computer.

## Compiling the repo
Ivan has prepared [a guide for this step in advance.](https://docs.google.com/document/d/1K5jpB43wlONjTTaYTXrp8nTQhextp7nA/edit?usp=sharing&ouid=107743509189700247789&rtpof=true&sd=true) Please give it a try if this step of the tutorial does not work.  

This repo uses PlatformIO on VSCode to compile the driver.  If you're familiar with other IDEs like Visual Studio, JetBrains, NetBeans, etc It's kinda like one of those.  PlatformIO is an extention to VSCode, which can be found at [https://code.visualstudio.com/](https://code.visualstudio.com/)  Once VSCode is installed, you can install PlatformIO here: [https://platformio.org/install/ide?install=vscode](https://platformio.org/install/ide?install=vscode)

Once PlatformIO is installed, their ant logo should appear on the sidebar on the left of VSCode.  If PIO's home page did not open, it can be found under Quick Access in the lower left if you select the logo.  Open the project by selecting open, and navigating to `platformio.ini` within the cloned repo.  

I'm not sure if PlatformIO needs to install the necisary libraries to build the driver.  I dont have a guinea pig windows computer to test this process. (2023 October 5)

You'll likely need to install Teensyduino. [https://www.pjrc.com/teensy/td_download.html](https://www.pjrc.com/teensy/td_download.html)

Assuming everything is working correctly, you should be ready to go to build the project.  We are currently using `ALARAV2_1_teensy36_Renegade` as the build target.
1. Click the > arrow next to `ALARAV2_1_teensy36_Renegade` to open the project folder
2. Click the > arrow next to `General`
3. Click on `Build` to build the project.  If it shows something similar to this, you successfully built the project!
```bash
Advanced Memory Usage is available via "PlatformIO Home > Project Inspect"
RAM:   [=         ]  11.9% (used 31076 bytes from 262144 bytes)
Flash: [===       ]  32.7% (used 342616 bytes from 1048576 bytes)
Building .pio\build\ALARAV2_1_teensy36_Renegade\firmware.hex
================ [SUCCESS] Took 8.18 seconds ================

Environment                  Status    Duration
---------------------------  --------  ------------
ALARAV2_1_teensy36_Renegade  SUCCESS   00:00:08.176
================ 1 succeeded in 00:00:08.176 ================
```

At this point you can alter the code and validate that it builds.  At the time of writing, we have no way of performing unit tests, but it is quite high on our todo list. (2023 October 5)
Please ask an Avionics team lead on the Discord if you have any questions, or if you'd like to commit changes to the repo!

## Navigating the repo
Git is a relatively complex tool. Don't be afraid to ask Avionics team members, Google, StackOverflow, or ChatGPT for help here.  There are loads of excellent tutorials for Git available online, if you need them.  

Beware of doing weirder things, as Git has a lot of "safety off" commands that can permanently delete your work.  There are some workarounds to recovering it, but be cautious unless you know what you're doing.

Before you continue, know that Git is not "smart."  It does not automatically update changes on Github locally, nor local changes to Github.  You must tell git to do both of these things.  

Here are a few useful commands for new Git users:
- `git branch` This lists all current branches on the repo.
- `git branch main` This changes the branch to the main branch, and updates your local files to reflect the contents of that branch.  If you wrote a branch that did not exist, git creates it for you. Beware of spelling errors!
- `git pull origin <main>` This will download remote changes from `origin`, which should have been automatically set to the `.git` repo name on Github you copied earlier. (2023 October 5)
- `git checkout` This is a complicated command that allows you to view past commits.  Please be careful with this one.
- `git diff` Compares uncommitted changes to the currently checked out commit.  Press `q` to cancel the command.
- `git add <file>` Marks a file's changes to be added to the next commit.
- `git log` Displays the commit history of the current branch.  Press `q` to cancel the command. 
- `git log -<n>` If you replace `<n>` with an integer, for example `git log -3`, git log will only show the last `n` commits.
- `git status` Lists uncommitted changes to the current branch. Very useful before you run:
- `git commit -m "<Descriptive, consice commit name>"` This adds a commit to the current branch.  Be careful performing a commit before you perform a pull, or you may encounter a conflict when pushing changes to Github.
 
 Some programmers are very picky about commit names, this is a good place to practice if you choose to be picky.  Please do not write paragraphs here, these are listed on the log.  If you omit `-m`, your default text editor will open a verbose file with a list of changes among other commands.  Saving the file completes the command.  I'm unfamiliar with this option and usually choose `-m` to avoid it due to its complexity.  

## Pushing changes to Github
Lets say you've made a branch, committed changes, and you want to merge it into the repo.  There are a few steps you need to do before you can commit changes:
1. You need to have a Github account.
2. You need to be logged into your Github account on your browser.
3. Your commit must build without errors.
4. You have not committed to Main directly.
5. You have not committed to Main directly.
6. You have not committed to Main directly.

Once you've done that, perform a git push:
 - `git push origin <branch>`
# Do NOT commit to main

Committing to main is a grave sin among Git users.  There's nothing stopping you (Git is in general "safety off") but it's considered extremely bad form.  The main branch is often assumed to be the rock solid foundation of a project, which should be polished and easy to branch changes from.  Committing untested, unreviewed code to main is quite dangerous.  

If you did not commit to main, head to Github on your browser and submit a "Pull Request" to the branch you want to merge into.  Once your code is reviewed, the Admin of the Github can merge your commits onto the branch.  This is how you commit to Main properly!

However, you don't have to commit to *only* main, the power of Git stems from being able to collaborate with other programmers by merging to other branches to build larger and larger features, and only merging with a higher branch when ready.  Some programmers prefer to use fancy history rewriting commands like `git rebase` but please be careful using this.

I hope this tutorial proved useful.  It is not a comprehensive intro to Git nor Github, and I highly suggest you look up more info on your own.  It is a very powerful tool that many programmers never master.  I cerntainly haven't!  Feel free to `git blame` me for any misleading information here.

🚀 Joe Kessler, 2023 October 5