'use strict'

const express = require('express')
const router = express.Router()

const csgrant = require('cloudsim-grant')

function setRoutes(app) {

  console.log('SIMULATIONS setRoutes')

  // router.route('/simulations')
  app.get('/simulations',function(req, res) {
    console.log('GET all simulations')
    const userToken = req.query.token
    console.log('   req.query.token: ' + userToken)
    csgrant.readAllResourcesForUser(userToken, (err, items) => {
      const r = {success: false}
      if(err) {
        r.error = err
      }
      else {
        r.success = true
        r.result = items
      }
      res.jsonp(r)
    })
  })

  // create a new simulation
  app.post('/simulations', function(req, res) {
    console.log('create sim: ' + JSON.stringify(req.body))
    // step 1, user verify
    const token = req.body.token
    csgrant.verifyToken(token, (err, decoded) => {
      if(err) {
        return res.jsonp( {success: false, error: err})
      }
      const user = decoded.username
      // step 2.  is user allowed?
      csgrant.isAuthorized(user, 'sim_list', false, (err, authorized) => {
        if(err) {
          return res.jsonp({success: false, error: err})
        }
        if(!authorized){
          const msg = 'insufficient permission for user "' + user + '"'
          return res.jsonp({success: false, error: msg})
        }
        // step 3. get unique id
        const r = {success: false}
        csgrant.getNextResourceId('sim', (err, id) => {
          const r = {success: false}
          if(err) {
            r.error = err
            res.jsonp(r)
            return
          }
          // step 4. add simulation
          csgrant.createResource(user, resource, data, (err, data) => {
            if(err) {
              r.error = err
              res.jsonp(r)
              return
            }
            r.success = true
            r.result = data
            r.id = id
            // success
            res.jsonp(r)
          })
        })
      })
    })
  })

  // read a simulation
  app.get('/simulations/:simId', function(req, res) {
    const resourceName = req.simId
    console.log('read /simulations/:simId = ' + resourceName)
    const token = req.body.token
    csgrant.verifyToken(token, (err, decoded) => {
      if(err) {
        return res.jsonp( {success: false, error: err})
      }
      const user = decoded.username
      // step 2.  is user allowed (read only)?
      csgrant.isAuthorized(user, resourceName, true, (err, authorized) => {
        if(err) {
          return res.jsonp({success: false, error: err})
        }
        if(!authorized){
          const msg = 'insufficient permission for user "' + user + '"'
          return res.jsonp({success: false, error: msg})
        }
        // step 3. get simulation
        const r = {success: false}
        csgrant.getResource(user, resource, (err, data) => {
          if(err) {
            return res.jsonp({success: false, error: err})
          }
          r.success = true
          r.result = data
          // success
          res.jsonp(r)
        })
      })
    })
  })

  // Update a simulation
  app.put('/simulations/:simId', function(req, res) {
    const resourceName = req.simId
    const newData = req.data
    console.log('Update simulation: ' + resourceName)
    console.log(' new data: ' + newData)

    const token = req.body.token
    csgrant.verifyToken(token, (err, decoded) => {
      if(err) {
        return res.jsonp( {success: false, error: err})
      }
      const user = decoded.username
      // step 2.  is user allowed (write perm) ?
      csgrant.isAuthorized(user, resourceName, false, (err, authorized) => {
        if(err) {
          return res.jsonp({success: false, error: err})
        }
        if(!authorized){
          const msg = 'insufficient permission for user "' + user + '"'
          return res.jsonp({success: false, error: msg})
        }
        // step 3. get simulation
        const r = {success: false}
        csgrant.updateResource(user, resource, newData, (err, data) => {
          if(err) {
            return res.jsonp({success: false, error: err})
          }
          r.success = true
          r.result = data
          // success
          res.jsonp(r)
        })
      })
    })
  })

  // Delete a simulation
  app.delete('/simulations/:simId', function(req, res) {
    console.log('delete simulation ' + req.simId)
    const resourceName = req.simId
    const newData = req.data
    console.log('Update simulation: ' + resourceName)
    console.log(' new data: ' + newData)

    const token = req.body.token
    csgrant.verifyToken(token, (err, decoded) => {
      if(err) {
        return res.jsonp( {success: false, error: err})
      }
      const user = decoded.username
      // step 2.  is user allowed (write perm) ?
      csgrant.isAuthorized(user, resourceName, false, (err, authorized) => {
        if(err) {
          return res.jsonp({success: false, error: err})
        }
        if(!authorized){
          const msg = 'insufficient permission for user "' + user + '"'
          return res.jsonp({success: false, error: msg})
        }
        // step 3. get simulation
        const r = {success: false}
        csgrant.deleteResource(user, resource, (err, data) => {
          if(err) {
            return res.jsonp({success: false, error: err})
          }
          r.success = true
          r.result = data
          // success
          res.jsonp(r)
        })
      })
    })
  })

  // simId
  app.param('simId', function( req, res, next, id) {
    console.log('   Resolving sim id: ' + id)

    if (true) {
      req.simId = id
    }
    next()
  })
}

exports.setRoutes = setRoutes
