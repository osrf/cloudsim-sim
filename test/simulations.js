'use strict';

console.log('test/mocha/simulations.js');

const should = require('should');
const supertest = require('supertest');
const csgrant = require('cloudsim-grant')
const token = csgrant.token

const app = require('../server/cloudsim_sim')
const agent = supertest.agent(app)

// we need fresh keys for this test
const keys = csgrant.token.generateKeys()
token.initKeys(keys.public, keys.private)

const admin = process.env.CLOUDSIM_ADMIN

console.log('admin user:', admin)

const adminTokenData = {
  identities: [admin]
}

let adminToken

const bobTokenData = {identities: ['bob']}
let bobToken


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
    setTimeout(function(){
      done()
    }, 1500);
  })

  before(function(done) {
    if (!admin || admin === "")
      should.fail('Admin user not specified')
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

  describe('Create a sim', function() {
    it('should be possible for admin to create a simulation', function(done) {
      agent
      .post('/simulations')
      .set('Acccept', 'application/json')
      .set('authorization', adminToken)
      .send({ cmd: 'ls -l',
              auto: true,
            })
      .end(function(err,res){
        res.status.should.be.equal(200)
        res.redirect.should.equal(false)
        var response = parseResponse(res.text)
        simId1 = response.id
        response.success.should.equal(true)
        if (!response.id.startsWith('simulation-')) {
          should.fail(response.id + ' is not a simualtion name')
        }
        response.requester.should.equal(admin)
        response.result.data.cmd.should.equal('ls -l')
        done()
      })
    })
  })

  // get resource
  describe('Get resource', function() {
    it('should be not possible for bob to get sim', function(done) {
      agent
      .get('/simulations/' + simId1)
      .set('Acccept', 'application/json')
      .set('authorization', bobToken)
      .send({})
      .end(function(err,res){
        res.status.should.be.equal(401)
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
      .send({ resource: simId1,
              grantee: 'bob',
              readOnly: true})
      .end(function(err,res){
        res.status.should.be.equal(200)
        res.redirect.should.equal(false)
        var response = parseResponse(res.text, true)
        response.success.should.equal(true)
        response.resource.should.equal(simId1)
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

  after(function(done) {
    csgrant.model.clearDb()
    done()
  })


})
