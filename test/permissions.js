'use strict';

console.log('test/mocha/permissions.js');

const util = require('util');
const should = require('should');
const supertest = require('supertest');
const csgrant = require('cloudsim-grant')
const token = csgrant.token

const app = require('../server/cloudsim_sim')
const agent = supertest.agent(app)

// we need fresh keys for this test
const keys = csgrant.token.generateKeys()
token.initKeys(keys.public, keys.private)

const admin = process.env.ADMIN_USER
const adminTokenData = {
    identities: [admin]
  }

let adminToken

const bobTokenData = {identities: ['bob']}
let bobToken


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

describe('<Unit test Permissions>', function() {
  before(function(done) {
    setTimeout(function(){
      done()
    }, 1000);
  })

  before(function(done) {
    csgrant.model.clearDb()
    csgrant.init(admin,
      {'sim-1':{data:'new data'}},
      'cloudsim-sim-test',
      'localhost',
      function(e) {
        if(e) {
          should.fail(e)
        }
        console.log('init')
      done()
    })
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
        response.result[2].name.should.equal('sim-1')
        done()
      })
    })
  })

  // give user read permission to sim1
  describe('Grant Read Permission', function() {
    it('should be possible to grant bob read permission for sim-1', function(done) {
      agent
      .post('/permissions')
      .set('Acccept', 'application/json')
      .set('authorization', adminToken)
      .send({ resource: 'sim-1',
              grantee: 'bob',
              readOnly: true})
      .end(function(err,res){
        res.status.should.be.equal(200)
        res.redirect.should.equal(false)
        var response = JSON.parse(res.text)
        response.success.should.equal(true)
        response.resource.should.equal('sim-1')
        response.requester.should.equal(admin)
        response.grantee.should.equal('bob')
        response.readOnly.should.equal(true)
        done()
      })
    })
  })

  // get resource
  describe('Get resource', function() {
    it('should be possible for bob to get sim-1', function(done) {
      agent
      .get('/simulations/sim-1')
      .set('Acccept', 'application/json')
      .set('authorization', bobToken)
      .send({})
      .end(function(err,res){
        res.status.should.be.equal(200)
        res.redirect.should.equal(false)
        var response = JSON.parse(res.text)
        response.success.should.equal(true)
        response.requester.should.equal('bob')
        response.result.name.should.equal('sim-1')

        response.result.permissions.length.should.equal(2)
        response.result.permissions[0].username.should.equal('bob')
        response.result.permissions[0].permissions.readOnly.should.equal(true)
        response.result.permissions[1].username.should.equal(admin)
        response.result.permissions[1].permissions.readOnly.should.equal(false)
        done()
      })
    })
  })

  // give user read permission to sim1
  describe('Revoke Read Permission', function() {
    it('should be possible to revoke user read permission', function(done) {
      agent
      .delete('/permissions')
      .set('Acccept', 'application/json')
      .set('authorization', adminToken)
      .send({ resource: 'sim-1',
              grantee: 'bob',
              readOnly: true})
      .end(function(err,res){
        res.status.should.be.equal(200)
        res.redirect.should.equal(false)
        var response = JSON.parse(res.text)
        response.success.should.equal(true)
        response.resource.should.equal('sim-1')
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
      .get('/simulations/sim-1')
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

})
