Flyingros\_web
=========

Flyingros\_web package allows the user to control, configure and monitor the multicopter from the web.

Installation
-------------

Flyingros\_web depends on Rosbridge\_server, which can be installed via the packages `ros-indigo-rosbridge-*`

To build the website, you need NodeJS, NPM and gulp. This is as a dependency for the simplicity of development. 

Then go into the [www](www) folder.

´´´bash
npm install -g gulp
npm install
gulp build
´´´

The output is in the `dist` folder. Open `index.html` in your browser.

[!Website image](images/website.png)

Available nodes 
-----------

* rtk_csv - Log the rtk (odometry) data in a CSV
* web_export - Send every usefull informations on the web though ROSBridge (to avoid to subscribe to 10 topics from the web app)

