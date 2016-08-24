'use strict'

const express = require('express')
const router = express.Router()
const csgrant = require('cloudsim-grant')

function setRoutes(app) {

  console.log('SIMULATIONS setRoutes')

  // list all resources
  app.get('/simulations',
    csgrant.authenticate,
    csgrant.ownsResource('simulation_list', false),
    csgrant.allResources)

  app.get('/simulations/:simId',
    csgrant.authenticate,
    csgrant.ownsResource(':simId', true),
    csgrant.resource)

  // create a new simulation
  app.post('/simulations',
           csgrant.authenticate,
           csgrant.ownsResource('simulation_list', false),
           function(req, res) {

    console.log('create sim:')
    console.log('  body:' +  JSON.stringify(req.body))
    console.log('  query:' + JSON.stringify(req.query))

    const data = req.query
    const resourceData = { cmd: data.cmd,
                           auto: data.auto,
                           stat:'WAITING'
                          }
    const error = function(msg) {
      return {operation: 'createSimulation',
              success: false,
              error: msg}
    }
    const op = 'createSimulation'
    const user = req.user
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

  // Update a simulation
  app.put('/simulations/:simId',
          csgrant.authenticate,
          csgrant.ownsResource(':simId', true),
          function(req, res) {

    const resourceName = req.simId
    const newData = req.query
    console.log(' Update simulation: ' + resourceName)
    console.log(' new data: ' + JSON.stringify(newData))
    const user = req.user

    const r = {success: false}
    if (!newData.cmd) {
       return res.jsonp({success: false, error: 'invalid new simulation: missing cmd'})
    }
    if (!newData.auto) {
       return res.jsonp({success: false, error: 'invalid new simulation: missing auto'})
    }

    csgrant.readResource(user, resourceName, function(err, oldData) {
      if(err)
        return res.jsonp({success: false, error: 'error trying to read existing data: ' + err})
      const futureData = oldData.data
      // merge with existing fields of the newData... thus keeping old fields intact
      for (var attrname in newData) { futureData[attrname] = newData[attrname] }
      csgrant.updateResource(user, resourceName, futureData, (err, data) => {
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

  // Delete a simulation
  app.delete('/simulations/:simId',
             csgrant.authenticate,
             csgrant.ownsResource(':simId', false),
             function(req, res) {
    console.log('delete simulation ' + req.simId)
    const resourceName = req.simId
    const r = {success: false}
    const user = req.user  // from previous middleware
    const resource = req.simId // from app.param (see below)
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

  // simId
  app.param('simId', function( req, res, next, id) {
    req.simId = id
    next()
  })
}

exports.setRoutes = setRoutes
