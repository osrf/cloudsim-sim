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
    console.log('  req.query.token: ' + userToken)
    csgrant.readAllResourcesForUser(userToken, (err, items) => {
      const r = {success: false, operation: 'getSimulations'}
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

    console.log('create sim:')
    console.log('  body:' +  JSON.stringify(req.body))
    console.log('  query:' + JSON.stringify(req.query))

    // step 1, user verify
    const token = req.query.token

    // The data for the new
    const resourceData = {cmd: req.query.cmd}

    const error = function(msg) {
      return {operation: 'createSimulation',
              success: false,
              error: msg}
    }
    const op = 'createSimulation'
    csgrant.verifyToken(token, (err, decoded) => {
      if(err) {
        return res.jsonp( error('invalid token'))
      }
      const user = decoded.username
      // step 2.  is user allowed?
      csgrant.isAuthorized(user, 'simulation_list', false,
                          (err, authorized) => {
        if(err) {
          return res.jsonp(error(err))
        }
        if(!authorized){
          const msg = 'insufficient permission for user "' + user + '"'
          return res.jsonp(error(msg))
        }
        // step 3. get unique id
        const r = {success: false}
        csgrant.getNextResourceId('sim', (err, resourceName) => {
          if(err) {
            res.jsonp(error(err))
            return
          }
          // step 4. add simulation
          csgrant.createResource(user, resourceName, resourceData,
                                (err, data) => {
            if(err) {
              res.jsonp(error(err))
              return
            }
            // step 5. success!
            const r = { success: true,
                        operation: op,
                        result: data,
                        id: resourceName}
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
