'use strict'

const csgrant = require('cloudsim-grant')



// Sets the routes for downloading the keys
// app: the express app
// keysFilePath: the full path to the keys.zip file
function setRoutes(app) {
  console.log('DOWNLOADS setRoutes')
  // user must have read only access to the
  // 'downloads' resource
  const resourceName = 'downloads'

csgrant.showLog = true

  app.get('/keys.zip',
    csgrant.authenticate,
    csgrant.ownsResource('downloads', false),
    function(req,res, next) {

      req.fileInfo = { path: req.resourceData.data.path ,
                       type: 'application/zip',
                       name: 'keys.zip'
                     }
      next()
    },
    csgrant.downloadFilePath)
}


exports.setRoutes = setRoutes
