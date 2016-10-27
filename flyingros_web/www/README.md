Web interface 
====================

To compile the web interface, you need to install Node, NPM and gulp. You can host it anywhere as it uses Rosbridge.

The compiled website is located in `flyingros_web/www/dist` as a static website, minified (and full) css, minified and concat (and full) js. 

```
sudo apt-get install node

cd flyingros_web/www
npm install -g gulp
npm install 
gulp
```

To host it, you may want to use NGINX or simply open the `index.html` file with your favorite browser.