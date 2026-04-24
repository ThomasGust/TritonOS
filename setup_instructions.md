The Pi should already be flashed with the code and configured the way it needs to be. If you ever find the code is broken or not accepting updates, delete the existing project folder on the ROV and run through the steps in reinstall.md in order to get the software working again.

This file will deal with the network setup instructions for windows (haven't been able to test this for a mac device yet). First of all, you will need to clone the pilot control repository.

The other things you need to download in order for things to work right are PuTTY which allows cleaner ssh into a terminal on the pi should anything need to be debugged outside the pilot software and dhcpsrv to handle the addressing.

Download from dhcpserver.de and putty.org.

I am not done writing everything out fully, but for the most part you should be able to follow the instructions at (THIS LINK)[https://www.youtube.com/watch?v=oM2zVD9rL8I] to set up ethernet communication between a desktop and the pi. For now, just use Griffin's computer or call/text me if you have pressing questions.