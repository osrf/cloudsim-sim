'use strict';

console.log('test/mocha/permissions.js');

const util = require('util');
const should = require('should');
const supertest = require('supertest');
const csgrant = require('cloudsim-grant')
const token = csgrant.token

const app = require('../../server/server')

const agent = supertest.agent(app)

// let's seed the database with a "sim-1"
csgrant.init('me', {'sim-1':{}}, 'cloudsim-sim-test', function() {
  console.log('init')
})

// we need fresh keys for this test
const keys = csgrant.token.generateKeys()
token.initKeys(keys.public, keys.private)

const meTokenData = {username:'me'}
let meToken



describe('<Unit test Permissions>', function() {

  before(function(done) {
      csgrant.model.clearDb()
      token.signToken({username: 'me'}, (e, tok)=>{
        console.log('token signed for user "me"')
        if(e) {
          console.log('sign error: ' + e)
        }
        meToken = tok
        done()
      })
  })

  // give user read permission to sim1
  describe('Grant Read Permission', function() {
    it('should be possible to grant user read permission', function(done) {
      agent
      .post('/permissions')
      .set('Acccept', 'application/json')
      .set('authorization', meToken)
      .send({ resource: 'sim-1',
              grantee: 'user',
              readOnly: true})
      .end(function(err,res){
        res.status.should.be.equal(200)
        res.redirect.should.equal(false)
        var response = JSON.parse(res.text)
        response.success.should.equal(true)
        response.resource.should.equal('sim-1')
        response.granter.should.equal('me')
        response.grantee.should.equal('user')
        response.readOnly.should.equal(true)
        done()
      })
    })
  })
})
