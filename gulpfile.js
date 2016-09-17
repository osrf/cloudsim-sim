'use strict';

var gulp = require('gulp');
var browserSync = require('browser-sync');
var nodemon = require('gulp-nodemon');
var mocha = require('gulp-mocha');

gulp.task('default', ['serve'], function () {
});

gulp.task('serve', ['nodemon'], function () {
});

gulp.task('browser-sync', ['nodemon'], function() {
	browserSync.init(null, {
//      middleware: [
//
//      ]

//     socket: {
//      domain: "localhost:5000"
//    }

//		proxy: "http://localhost:5000",
//        files: ["public/**/*.*"],
//        browser: "google chrome",
//        port: 7000,
	});
});

gulp.task('nodemon', function (cb) {

	var started = false;

	return nodemon({
		script: 'server/server.js',
    watch: ['server/**/*.*']
	}).on('start', function () {
    console.log('start nodemon')
		// to avoid nodemon being started multiple times
		// thanks @matthisk
		if (!started) {
			cb();
			started = true;
		}
	});
});

gulp.task('test', function() {
  return gulp.src(['test/**/*.js'], {read: false})
    .pipe(mocha({
      reporter: 'spec'
    }))
    .once('end', function () {
      process.exit();
    });
});
