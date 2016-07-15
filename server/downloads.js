'use strict'

const fs = require('fs')
const express = require('express')
const router = express.Router()
const csgrant = require('cloudsim-grant')
const path = require('path')

// Sets the routes for downloading the keys
// app: the express app
// keysFilePath: the full path to the keys.zip file
function setRoutes(app) {
  console.log('DOWNLOADS setRoutes')
  // user must have read only access to the
  // 'downloads' resource
  const resourceName = 'downloads'
  // read a simulation
  app.get('/keys.zip', function(req, res) {
    console.log('\ndownload: keys.zip ')
    const token = req.query.token
    csgrant.verifyToken(token, (err, decoded) => {
      if(err) {
        console.log(' verify token fail [' + err + ']')
        return res.jsonp({success: false,
                          error: "authorization error: no token provided"
                         })
      }
      const user = decoded.username
      // step 2.  is user allowed (read only)?
      csgrant.isAuthorized(user, resourceName, true, (err, authorized) => {
        if(err) {
          console.log (' authorization error: [' + err + ']')
          return res.jsonp({success: false, error: err})
        }
        if(!authorized){
          const msg = 'insufficient permission for user "' + user + '"'
          console.log( ' ' + msg)
          return res.jsonp({success: false, error: msg})
        }
        // step 3. serve file
        const r = {success: false}
        csgrant.readResource(user, resourceName, (err, resource) => {
          if(err) {
            console.log(' get resource error: ' + err)
            return res.jsonp({success: false, error: err})
          }
          console.log(' sucess: '+ JSON.stringify(resource))
          downloadFile(resource.data.path, res)
          // success
          // return res.jsonp({success: true, result: data})
        })
      })
    })
  })
}


//////////////////////////////////////////////////////
// local  utility function to download
// a single file. Used for keys.zip
// @param path the path of the local file to download
//
function downloadFile(filePath, res) {

  fs.stat(filePath, function(err, stat) {
    if(err) {
        if('ENOENT' === err.code) {
            res.statusCode = 404;
            console.log('file "' + filePath + '" not found')
            return res.end('Not Found');
        }
    } else {
      res.setHeader('Content-type', 'application/zip')
      const f = path.basename(filePath)
      res.setHeader('Content-disposition', 'attachment; filename=' + f)
      res.setHeader('Content-Length', stat.size)
      var stream = fs.createReadStream(filePath);
      stream.pipe(res);
      stream.on('error', function() {
          res.statusCode = 500;
          console.log('Error downloading file "' + filePath + '"')
          res.end('Internal Server Error');
      });
      stream.on('end', function() {
          console.log('"' + filePath + '" downloaded')
          res.end();
      });
    }
  });
}

exports.setRoutes = setRoutes
