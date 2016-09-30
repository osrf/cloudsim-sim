'use strict';

console.log('test/permissions.js');

const should = require('should');
const supertest = require('supertest');
const csgrant = require('cloudsim-grant')

const app = require('../server/cloudsim_sim')
const token = csgrant.token

const agent = supertest.agent(app)

// we need fresh keys for this test
const keys = csgrant.token.generateKeys()

token.initKeys(keys.public, keys.private)

const admin = process.env.CLOUDSIM_ADMIN || "admin"
const adminTokenData = {
    identities: [admin]
  }

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
    token.initKeys(keys.public, keys.private)
    done()
  })

  before(function(done) {
    token.signToken(adminTokenData, (e, tok)=>{
      console.log('token signed for "' + admin + '"')
      if(e) {
        console.log('sign error: ' + e)
      }
      adminToken = tok
      token.signToken(bobTokenData, (e, tok)=>{
        console.log('token signed for user "bob"')
        if(e) {
          console.log('sign error: ' + e)
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
        var response = parseResponse(res.text)
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
      .send({ auto: false})
      .end(function(err,res){
        res.status.should.be.equal(200)
        res.redirect.should.equal(false)
        const response = parseResponse(res.text)
        response.success.should.equal(true)
console.log('--------------------')
console.log(response)
console.log('--------------------')
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
    done()
  })

})
