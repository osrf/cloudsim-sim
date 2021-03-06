'use strict';

console.log('test/permissions.js');

const should = require('should');
const supertest = require('supertest');
const clearRequire = require('clear-require');

let csgrant
let app
let agent
let token

let admin
let adminTokenData
let adminToken

const bobTokenData = {identities: ['bob']}
let bobToken

function parseResponse(text, log) {
  if(log) {
    csgrant.dump()
  }
  let res
  try {
    res = JSON.parse(text)
  }
  catch (e) {
    console.log(text)
    throw e
  }
  if(log){
    const s = JSON.stringify(res, null, 2)
    console.log(s)
  }
  return res
}

describe('<Unit test Permissions>', function() {

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
    token = csgrant.token
    done()
  })

  before(function(done) {
    admin = process.env.CLOUDSIM_ADMIN || "admin"
    if (!admin || admin === "") {
      should.fail('Admin user not specified')
    }
    adminTokenData = { identities: [admin] }

    token.signToken(adminTokenData, (e, tok) => {
      console.log('token signed for "' + admin + '"')
      if(e) {
        should.fail('sign error: ' + e)
      }
      adminToken = tok
      token.signToken(bobTokenData, (e, tok)=>{
        console.log('token signed for user "bob"')
        if(e) {
          should.fail('sign error: ' + e)
        }
        bobToken = tok
        done()
      })
    })
  })

  // create a sim
  let simId
  describe('Create a sim', function() {
    it('should be possible to create a simulation', function(done) {
      agent
      .post('/simulations')
      .set('Acccept', 'application/json')
      .set('authorization', adminToken)
      .send({ cmd: 'ls -l',
        auto: true,
      })
      .end(function(err,res){
        console.log(err)
        var response = parseResponse(res.text, res.status != 200)
        res.status.should.be.equal(200)
        res.redirect.should.equal(false)
        response.success.should.equal(true)
        simId = response.id
        if(!simId.startsWith("simulation-")) {
          should.fail('"'+ simId + +'" invalid sim name')
        }
        response.requester.should.equal(admin)
        done()
      })
    })
  })

  // get all resources
  describe('Get all resources', function() {
    it('should be possible for admin to get all resources', function(done) {
      agent
      .get('/permissions')
      .set('Acccept', 'application/json')
      .set('authorization', adminToken)
      .send({})
      .end(function(err,res){
        const response = parseResponse(res.text)
        res.status.should.be.equal(200)
        res.redirect.should.equal(false)
        response.success.should.equal(true)
        response.requester.should.equal(admin)
        response.result.length.should.equal(3)
        response.result[0].name.should.equal('simulations')
        response.result[1].name.should.equal('downloads')
        response.result[2].name.should.equal(simId)
        done()
      })
    })
  })

  // get resource fail
  describe('Get resource fail', function() {
    it('should not be possible for bob to get sim', function(done) {
      agent
      .get('/simulations/' + simId)
      .set('Acccept', 'application/json')
      .set('authorization', bobToken)
      .send({})
      .end(function(err,res){
        res.status.should.be.equal(401)
        res.redirect.should.equal(false)
        var response = JSON.parse(res.text)
        response.success.should.equal(false)
        should.exist(response.error)
        done()
      })
    })
  })

  // give user read permission to sim1
  describe('Grant Read Permission', function() {
    it('should be possible to grant bob read permission for sim', function(done) {
      agent
      .post('/permissions')
      .set('Acccept', 'application/json')
      .set('authorization', adminToken)
      .send({ resource: simId,
        grantee: 'bob',
        readOnly: true})
      .end(function(err,res){
        res.status.should.be.equal(200)
        res.redirect.should.equal(false)
        const response = parseResponse(res.text)
        response.success.should.equal(true)
        response.resource.should.equal(simId)
        response.requester.should.equal(admin)
        response.grantee.should.equal('bob')
        response.readOnly.should.equal(true)
        done()
      })
    })
  })

  // get all resources for bob
  describe('Get all resources for bob', function() {
    it('should see sim', function(done) {
      agent
      .get('/permissions')
      .set('Acccept', 'application/json')
      .set('authorization', bobToken)
      .send({})
      .end(function(err,res){
        const response = parseResponse(res.text)
        res.status.should.be.equal(200)
        res.redirect.should.equal(false)
        response.success.should.equal(true)
        response.requester.should.equal('bob')
        response.result.length.should.equal(1)
        response.result[0].name.should.equal(simId)
        done()
      })
    })
  })

  // get resource
  describe('Get resource', function() {
    it('should be possible for bob to get sim', function(done) {
      agent
      .get('/simulations/' + simId)
      .set('Acccept', 'application/json')
      .set('authorization', bobToken)
      .send({})
      .end(function(err,res){
        res.status.should.be.equal(200)
        res.redirect.should.equal(false)
        var response = JSON.parse(res.text)
        response.success.should.equal(true)
        response.requester.should.equal('bob')
        response.result.name.should.equal(simId)

        response.result.permissions.length.should.equal(2)
        response.result.permissions[0].username.should.equal('bob')
        response.result.permissions[0].permissions.readOnly.should.equal(true)
        response.result.permissions[1].username.should.equal(admin)
        response.result.permissions[1].permissions.readOnly.should.equal(false)
        done()
      })
    })
  })

  // revoke read permission to sim1
  describe('Revoke Read Permission', function() {
    it('should be possible to revoke user read permission', function(done) {
      agent
      .delete('/permissions')
      .set('Acccept', 'application/json')
      .set('authorization', adminToken)
      .send({ resource: simId,
        grantee: 'bob',
        readOnly: true})
      .end(function(err,res){
        res.status.should.be.equal(200)
        res.redirect.should.equal(false)
        const response = parseResponse(res.text)
        response.success.should.equal(true)
        response.resource.should.equal(simId)
        response.requester.should.equal(admin)
        response.grantee.should.equal('bob')
        response.readOnly.should.equal(true)
        done()
      })
    })
  })

  // get resource
  describe('Get resource', function() {
    it('should not be possible for bob to get sim-1 anymore', function(done) {
      agent
      .get('/simulations/' + simId)
      .set('Acccept', 'application/json')
      .set('authorization', bobToken)
      .send({})
      .end(function(err,res){
        res.status.should.be.equal(401)
        var response = JSON.parse(res.text)
        response.success.should.equal(false)
        should.exist(response.error)
        done()
      })
    })
  })

  // update resource
  describe('Update resource', function() {
    it('should be possible for admin to update resource', function(done) {
      agent
      .put('/simulations/' + simId)
      .set('Acccept', 'application/json')
      .set('authorization', adminToken)
      .send({ 
        cmd: 'ls -la',
        stopCmd: 'ls -la',
        logCmd: 'ls -la',
        auto: false,
      })
      .end(function(err,res){
        res.status.should.be.equal(200)
        res.redirect.should.equal(false)
        const response = parseResponse(res.text)
        response.success.should.equal(true)
        done()
      })
    })
  })
  // update resource
  describe('Update resource', function() {
    it('should NOT be possible for admin to update resource if there are missing fields', function(done) {
      agent
      .put('/simulations/' + simId)
      .set('Acccept', 'application/json')
      .set('authorization', adminToken)
      .send({ auto: false})
      .end(function(err,res){
        res.status.should.be.equal(400)
        res.redirect.should.equal(false)
        done()
      })
    })
  })

  // delete resource
  describe('Delete simulation', function() {
    it('should be possible for admin to delete sim', function(done) {
      agent
      .delete('/simulations/' + simId)
      .set('Acccept', 'application/json')
      .set('authorization', adminToken)
      .send({})
      .end(function(err,res){
        res.status.should.be.equal(200)
        var response = JSON.parse(res.text)
        response.success.should.equal(true)
        done()
      })
    })
  })

  // check that sim is not in all resources
  describe('Get all resources for admin', function() {
    it('should not have sim anymore', function(done) {
      agent
      .get('/permissions')
      .set('Acccept', 'application/json')
      .set('authorization', adminToken)
      .send({})
      .end(function(err,res){
        const response = parseResponse(res.text)
        res.status.should.be.equal(200)
        res.redirect.should.equal(false)
        response.success.should.equal(true)
        response.requester.should.equal(admin)
        response.result.length.should.equal(2)
        response.result[0].name.should.equal('simulations')
        response.result[1].name.should.equal('downloads')
        done()
      })
    })
  })

  after(function(done) {
    csgrant.model.clearDb()
    app.close(function() {
      clearRequire.all()
      done()
    })
  })

})
