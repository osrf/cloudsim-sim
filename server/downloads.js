'use strict'
const csgrant = require('cloudsim-grant')

// Sets the routes for downloading the keys
// app: the express app
// keysFilePath: the full path to the keys.zip file
function setRoutes(app) {
  console.log('DOWNLOADS setRoutes')
  app.get('/keys.zip',
    // user must have valid token (in req.query)
    csgrant.authenticate,
    // user must have access to 'downloads' resource
    // this middleware will set req.resourceData
    csgrant.ownsResource('downloads', false),
    // this middleware sets the file download information from
    // the resource in req.resourceData
    function(req,res, next) {

      req.fileInfo = { path: req.resourceData.data.path ,
        type: 'application/zip',
        name: 'keys.zip'
      }
      next()
    },
    // with a req.fileInfo in place, this middleware will
    // download the file from the disk
    csgrant.downloadFilePath)
}

exports.setRoutes = setRoutes
