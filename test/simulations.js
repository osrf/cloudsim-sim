'use strict';

console.log('test/mocha/simulations.js');

const should = require('should');
const supertest = require('supertest');
const clearRequire = require('clear-require');

let csgrant
let app
let agent

let admin
let adminTokenData

let adminToken

// holds the simulation id
let simId1

function parseResponse(text, log) {
  let res
  try {
    res = JSON.parse(text)
  }
  catch (e) {
    console.log(text)
    throw e
  }
  if(log){
    csgrant.dump()
    const s = JSON.stringify(res, null, 2)
    console.log(s)
  }
  return res
}

describe('<Unit test Simulations>', function() {

  before(function(done) {
    // Important: the database has to be cleared early, before
    // the server is launched (otherwise, root resources will be missing)
    csgrant = require('cloudsim-grant')
    csgrant.model.clearDb()
    done()
  })

  before(function(done) {
    app = require('../server/cloudsim_sim')
    agent = supertest.agent(app)
    done()
  })

  before(function(done) {
    // we need fresh keys for this test
    const keys = csgrant.token.generateKeys()
    csgrant.token.initKeys(keys.public, keys.private)
    done()
  })

  before(function(done) {
    admin = process.env.CLOUDSIM_ADMIN || 'admin'
    if (!admin || admin === "") {
      should.fail('Admin user not specified')
    }
    console.log("admin user:", admin)

    adminTokenData = { identities: [admin] }

    csgrant.token.signToken(adminTokenData, (e, tok)=>{
      console.log('token signed for "' + admin + '"')
      if(e) {
        console.log('sign error: ' + e)
      }
      adminToken = tok
      done()
    })
  })

  // get all resources
  describe('Get all simulations', function() {
    it('there should be none', function(done) {
      agent
      .get('/simulations')
      .set('Acccept', 'application/json')
      .set('authorization', adminToken)
      .send({})
      .end(function(err,res){
        const response = parseResponse(res.text)
        res.status.should.be.equal(200)
        res.redirect.should.equal(false)
        response.success.should.equal(true)
        response.requester.should.equal(admin)
        response.result.length.should.equal(0)
        done()
      })
    })
  })

  // Create a simulation with all commands as arguments 
  describe('Create a sim', function() {
    it('should be possible for admin to create a simulation', function(done) {
      agent
      .post('/simulations')
      .set('Acccept', 'application/json')
      .set('authorization', adminToken)
      .send({ 
        cmd: 'ls -l',
        stopCmd: 'ls -la',
        logCmd: 'ls -lah',        
        auto: true,
      })
      .end(function(err,res){
        res.status.should.be.equal(200)
        res.redirect.should.equal(false)
        var response = parseResponse(res.text)
        simId1 = response.id
        response.success.should.equal(true)
        if (!response.id.startsWith('simulation-')) {
          should.fail(response.id + ' is not a simulation name')
        }
        response.requester.should.equal(admin)
        response.result.data.cmd.should.equal('ls -l')
        response.result.data.stopCmd.should.equal('ls -la')
        response.result.data.logCmd.should.equal('ls -lah')
        response.result.data.auto.should.equal(true)
        done()
      })
    })
    it('there should be one sim', function(done) {
      agent
      .get('/simulations')
      .set('Acccept', 'application/json')
      .set('authorization', adminToken)
      .send({})
      .end(function(err,res){
        const response = parseResponse(res.text)
        res.status.should.be.equal(200)
        res.redirect.should.equal(false)
        response.success.should.equal(true)
        response.requester.should.equal(admin)
        response.result.length.should.equal(1)
        done()
      })
    })    
  })

  // Get simulation
  describe('Get resource', function() {
    it('should be possible for admin to get sim', function(done) {
      agent
      .get('/simulations/' + simId1)
      .set('Acccept', 'application/json')
      .set('authorization', adminToken)
      .send({})
      .end(function(err,res){
        should.not.exist(err)

        res.status.should.be.equal(200)
        res.redirect.should.equal(false)

        const response = parseResponse(res.text)
        response.success.should.equal(true)
        response.resource.should.equal(simId1)
        response.requester.should.equal(admin)
        response.result.data.cmd.should.equal('ls -l')
        response.result.data.auto.should.equal(true)

        done()
      })
    })
  })

  // Update simulation
  describe('Update a sim', function() {
    it('should be possible for admin to update a simulation', function(done) {
      agent
      .put('/simulations/' + simId1)
      .set('Acccept', 'application/json')
      .set('authorization', adminToken)
      .send({ 
        cmd: 'ls -la',
        auto: false,
      })
      .end(function(err,res){
        res.status.should.be.equal(200)
        res.redirect.should.equal(false)

        var response = parseResponse(res.text)
        response.success.should.equal(true)
        response.requester.should.equal(admin)
        response.id.should.equal(simId1)
        response.result.data.cmd.should.equal('ls -la')
        // other commands should have stayed the same
        response.result.data.stopCmd.should.equal('ls -la')
        response.result.data.logCmd.should.equal('ls -lah')
        response.result.data.auto.should.equal(false)
        done()
      })
    })
  })

  // Update simulation
  describe('Update a sim', function() {
    it('should be possible for admin to update a simulation with all commands', function(done) {
      agent
      .put('/simulations/' + simId1)
      .set('Acccept', 'application/json')
      .set('authorization', adminToken)
      .send({ 
        cmd: 'differentCommand1',
        stopCmd: 'my super stop script',
        logCmd: 'logme',
        auto: false,
      })
      .end(function(err,res){
        res.status.should.be.equal(200)
        res.redirect.should.equal(false)

        const response = parseResponse(res.text)
        response.success.should.equal(true)
        response.requester.should.equal(admin)
        response.id.should.equal(simId1)
        response.result.data.cmd.should.equal('differentCommand1')
        response.result.data.stopCmd.should.equal('my super stop script')
        response.result.data.logCmd.should.equal('logme')
        response.result.data.auto.should.equal(false)
        done()
      })
    })
    it('lets go back to some real commands', function(done) {
      agent
      .put('/simulations/' + simId1)
      .set('Acccept', 'application/json')
      .set('authorization', adminToken)
      .send({ 
        cmd: 'ls -la',
        stopCmd: 'ls -la',
        logCmd: 'ls -lah',
        auto: false,
      })
      .end(function(err,res){
        res.status.should.be.equal(200)
        res.redirect.should.equal(false)

        const response = parseResponse(res.text)
        response.success.should.equal(true)
        response.requester.should.equal(admin)
        response.id.should.equal(simId1)
        response.result.data.cmd.should.equal('ls -la')
        response.result.data.stopCmd.should.equal('ls -la')
        response.result.data.logCmd.should.equal('ls -lah')
        response.result.data.auto.should.equal(false)
        done()
      })
    })
  })

  // Update simulation
  describe('Update a sim', function() {
    it('should not be able to update sim if cmd or auto args are not sent', function(done) {
      agent
      .put('/simulations/' + simId1)
      .set('Acccept', 'application/json')
      .set('authorization', adminToken)
      .send({
        auto: false,
      })
      .end(function(err,res){
        res.status.should.be.equal(400)
        res.redirect.should.equal(false)
        const response = parseResponse(res.text)
        response.success.should.equal(false)
        // lets now try with no auto arg
        agent
        .put('/simulations/' + simId1)
        .set('Acccept', 'application/json')
        .set('authorization', adminToken)
        .send({ 
          cmd: 'ls -la'
        })
        .end(function(err,res){
          res.status.should.be.equal(400)
          res.redirect.should.equal(false)
          const response = parseResponse(res.text)
          response.success.should.equal(false)
          done()
        })
      })
    })
  })

  // Get to check update
  describe('Get resource', function() {
    it('should have the updated fields', function(done) {
      agent
      .get('/simulations/' + simId1)
      .set('Acccept', 'application/json')
      .set('authorization', adminToken)
      .send({})
      .end(function(err,res){
        should.not.exist(err)

        res.status.should.be.equal(200)
        res.redirect.should.equal(false)

        const response = parseResponse(res.text)
        response.success.should.equal(true)
        response.resource.should.equal(simId1)
        response.requester.should.equal(admin)
        response.result.data.cmd.should.equal('ls -la')
        response.result.data.auto.should.equal(false)

        done()
      })
    })
  })

  // Stop simulation
  describe('Stop simulation resource', function() {
    it('there should be one sim', function(done) {
      agent
      .get('/simulations')
      .set('Acccept', 'application/json')
      .set('authorization', adminToken)
      .send({})
      .end(function(err,res){
        const response = parseResponse(res.text)
        res.status.should.be.equal(200)
        res.redirect.should.equal(false)
        response.success.should.equal(true)
        response.requester.should.equal(admin)
        response.result.length.should.equal(1)
        done()
      })
    })    
    it('should be possible for admin to stop a simulation', function(done) {
      agent
      .post('/stopsimulation')
      .set('Acccept', 'application/json')
      .set('authorization', adminToken)
      .end(function(err,res){
        should.not.exist(err)
        res.status.should.be.equal(200)
        res.redirect.should.equal(false)
        const response = parseResponse(res.text)
        should.exist(response.state)
        should.exist(response.prior)
        done()
      })
    })
    it('stopped sim should be marked as finished (after a little while)', function(done) {
      setTimeout(() => {
        agent
        .get('/simulations/' + simId1)
        .set('Acccept', 'application/json')
        .set('authorization', adminToken)
        .send({})
        .end(function(err,res){
          should.not.exist(err)
          res.status.should.be.equal(200)
          res.redirect.should.equal(false)
          const response = parseResponse(res.text)
          response.success.should.equal(true)
          response.resource.should.equal(simId1)
          response.result.data.stat.should.equal('FINISHED')
          response.result.data.cmd.should.equal('ls -la')
          response.result.data.stopCmd.should.equal('ls -la')
          response.result.data.logCmd.should.equal('ls -lah')
          response.result.data.auto.should.equal(false)
          done()
        })
      }, 10)
    })
  })

  // Delete simulation
  describe('Delete resource', function() {
    it('should be possible for admin to delete simulation', function(done) {
      agent
      .delete('/simulations/' + simId1)
      .set('Acccept', 'application/json')
      .set('authorization', adminToken)
      .send({})
      .end(function(err,res){
        should.not.exist(err)

        res.status.should.be.equal(200)
        res.redirect.should.equal(false)

        const response = parseResponse(res.text)
        response.success.should.equal(true)
        response.id.should.equal(simId1)

        done()
      })
    })
  })

  // Start and stop simulation just passing the "start cmd" (ie. no stop nor log commands) 
  let simId2
  describe('Start / Stop a sim just with start cmd', function() {
    it('should be possible for admin to create a simulation just with start cmd (ie, no stop nor log cmds)', function(done) {
      agent
      .post('/simulations')
      .set('Acccept', 'application/json')
      .set('authorization', adminToken)
      .send({
        cmd: 'ls -l',        
        auto: true,
      })
      .end(function(err,res){
        res.status.should.be.equal(200)
        res.redirect.should.equal(false)
        var response = parseResponse(res.text)
        simId2 = response.id
        response.success.should.equal(true)
        response.requester.should.equal(admin)
        response.result.data.cmd.should.equal('ls -l')
        should.not.exist(response.result.data.stopCmd)
        should.not.exist(response.result.data.logCmd)
        response.result.data.auto.should.equal(true)
        done()
      })
    })
    it('should be possible for admin to stop a simulation', function(done) {
      agent
      .post('/stopsimulation')
      .set('Acccept', 'application/json')
      .set('authorization', adminToken)
      .end(function(err,res){
        should.not.exist(err)
        res.status.should.be.equal(200)
        res.redirect.should.equal(false)
        const response = parseResponse(res.text)
        should.exist(response.state)
        should.exist(response.prior)
        done()
      })
    })    
    it('stopped sim should be marked as finished (after a little while)', function(done) {
      setTimeout(() => {
        agent
        .get('/simulations/' + simId2)
        .set('Acccept', 'application/json')
        .set('authorization', adminToken)
        .send({})
        .end(function(err,res){
          should.not.exist(err)
          res.status.should.be.equal(200)
          res.redirect.should.equal(false)
          const response = parseResponse(res.text)
          response.success.should.equal(true)
          response.resource.should.equal(simId2)
          response.result.data.stat.should.equal('FINISHED')
          response.result.data.cmd.should.equal('ls -l')
          should.not.exist(response.result.data.stopCmd)
          should.not.exist(response.result.data.logCmd)
          response.result.data.auto.should.equal(true)
          done()
        })
      }, 10)
    })
    
  })

  // after all tests have run, we need to clean up our mess
  after(function(done) {
    csgrant.model.clearDb()
    app.close(function() {
      clearRequire.all()
      done()
    })
  })

})
