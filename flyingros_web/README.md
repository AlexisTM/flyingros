Flyingros\_web
=========

Flyingros\_web package allows the user to control, configure and monitor the multicopter from the web.

Installation
-------------

Flyingros\_web depends on Rosbridge\_server, which can be installed via the packages `ros-indigo-rosbridge-*`

To build the website, you need NodeJS, NPM and gulp. This is as a dependency for the simplicity of development. Once compiled, while you don't change the source, you can simply use the dist generated folder.

Then go into the `www` folder.

´´´bash
sudo apt-get install nodejs-legacy
sudo apt-get install npm
npm install -g gulp
npm install
gulp build # or simply gulp to develop 
´´´

The output is in the `dist` folder. Open `index.html` in your browser.

[!Website image](images/website.png)

Implemented functions
-----------

* Add a task 
* Remove a task (click on the task)
* Export/import mission
* Get the mission
* Remove all tasks

Available nodes 
-----------

* `rosrun flyingros_web web_export` - Send every usefull informations on the web though ROSBridge (to avoid to subscribe to 10 topics from the web app)

