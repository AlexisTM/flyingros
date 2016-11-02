'use strict';

var jade = require('jade');
var gulp = require('gulp');
var uglify = require('gulp-uglify');
var uglifycss = require('gulp-uglifycss');
var gulpJade = require('gulp-jade');
var concat = require('gulp-concat');
var pump = require('pump');
var livereload = require('gulp-livereload');
var serve = require('gulp-serve');

var paths = {
  jsconcat: ['src/config.js',
              'src/entry.js',
              'src/task.js',
              'src/ros.js',
              'src/dom.js',
              'src/form_validator.js',
              'src/paper.js',
              'src/start.js'],
  static: 'static/*',
  jslib: 'jslib/*'
};

gulp.task('js', bundleJS); // so you can run `gulp js` to build the file

gulp.task('jslib', bundleJSLIB); // so you can run `gulp js` to build the file

gulp.task('css', bundleCSS);

gulp.task('jade', bundleHTML)

gulp.task('watch', watch)

gulp.task('images', bundleStatic);

gulp.task('default', ['js','jslib','css','jade','images','watch']);

gulp.task('serve', serve('dist'));

gulp.task('sw', ['serve', 'watch']);

function watch(cb){
  gulp.watch('src/*.js', ['js']);
  gulp.watch('css/*.css', ['css']);
  gulp.watch('index.jade', ['jade']);
  livereload.listen();
}

function bundleCSS(cb){
  pump([gulp.src(['css/custom.css', 'css/font-awesome.min.css', 'css/modalise.min.css', 'css/font-awesome-animation.min.css']),
        concat('app.css'),
        gulp.dest('dist'),
        uglifycss({
          "uglyComments": true
        }),
        concat('app.min.css'),
        gulp.dest('dist'),
        livereload()], cb);
}

function bundleHTML(cb){
  pump([gulp.src('index.jade'),
        gulpJade({ jade: jade }),
        gulp.dest('dist'),
        livereload()], cb);
}


function bundleJS(cb) {
  pump([gulp.src(paths.jsconcat),
        concat('app.js'),
        gulp.dest('dist'),
        concat('app.min.js'),
        uglify(),
        gulp.dest('dist'),
        livereload()
       ], cb);
}

function bundleJSLIB(cb){
  pump([gulp.src(paths.jslib),
        concat('lib.min.js'),
        uglify(),
        gulp.dest('dist'),
        livereload()], cb)
}

function bundleStatic(cb){
  pump([gulp.src(paths.static),
        gulp.dest('dist/static')], cb);
}