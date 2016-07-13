'use strict'

const express = require('express')
const router = express.Router()
const csgrant = require('cloudsim-grant')

//
// Sets the routes for downloading the keys
//
function setRoutes(app) {
  console.log('DOWNLOADS setRoutes')
  // user must have read only access to the
  // 'downloads' resource
  const resourceName = 'downloads'
  // read a simulation
  app.get('/keys.zip', function(req, res) {
    console.log('download /simulations/' + resourceName + '/ssh')
    const token = req.body.token
    console.log(' token: ' + token)
    csgrant.verifyToken(token, (err, decoded) => {
      if(err) {
        console.log(' verify token fail ' + err)
        return res.jsonp( {success: false, error: err})
      }
      const user = decoded.username
      // step 2.  is user allowed (read only)?
      csgrant.isAuthorized(user, resourceName, true, (err, authorized) => {
        if(err) {
          console.log (' authorization err: ' + err)
          return res.jsonp({success: false, error: err})
        }
        if(!authorized){
          const msg = 'insufficient permission for user "' + user + '"'
          console.log( ' ' + msg)
          return res.jsonp({success: false, error: msg})
        }
        // step 3. serve file
        const r = {success: false}
        csgrant.getResource(user, resource, (err, data) => {
          if(err) {
            console.log(' get resource error: ' + err)
            return res.jsonp({success: false, error: err})
          }
          console.log(' sucess: '+ JSON.stringify(data))
          // success
          res.jsonp({success: true, result: data})
        })
      })
    })
  })
}

exports.setRoutes = setRoutes
