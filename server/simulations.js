'use strict'

const express = require('express')
const router = express.Router()

function setRoutes(app) {

  console.log('SIMULATIONS setRoutes')

  // router.route('/simulations')
  app.get('/simulations',function(req, res) {
    console.log('req: ' + req)
    console.log('GET simulations')
    const r =[]
    res.jsonp(r)
  })

  // create a new simulation
  app.post('/simulations', function(req, res) {
    const r = {success: false}

    console.log('Create sim: ' + JSON.stringify(req.body))

    r.simId = 42
    res.jsonp(r)
  })

  // read a simulation
  app.get('/simulations/:simId', function(req, res) {
    console.log('/simulations/:simId = ' + req.simId)
    console.log('type: ' + typeof(req.simId))
    const r = {success: false}
    res.jsonp(r)
  })

  // Update a simulation
  app.put('/simulations/:simId', function(req, res) {
    console.log('Update simulation ' + req.simId)
    const r = {success: false}
    res.jsonp(r)
  })

  // Delete a simulation
  app.delete('/simulations/:simId', function(req, res) {
    console.log('delete simulation ' + req.simId)
    const r ={success: false}
    res.jsonp(r)
  })

  // simId
  app.param('simId', function( req, res, next, id) {
    console.log('   Resolving sim id: ' + id)
    if (true) {
      req.simId = Number(id)
    }
    next()
  })
}

exports.setRoutes = setRoutes
