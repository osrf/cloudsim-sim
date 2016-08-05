'use strict';

console.log('test/mocha/permissions.js');

require('../../server/server.js')
var app = require('../../server/server')

var util = require('util');
var should = require('should');
var supertest = require('supertest');

var admin = 'admin';
var user = 'user';
var simId1 = 'sim-1';
var agent = supertest.agent(app);

describe('Permissions', function() {
  // give user read permission to sim1
  describe('Grant Read Permission', function() {
    it('should be possible to grant user read permission', function(done) {
      agent
      .post('/permissions')
      .set('Acccept', 'application/json')
      .send({id: simId1, username: user, read_only: true})
      .end(function(err,res){
        res.status.should.be.equal(200);
        res.redirect.should.equal(false);
        var text = JSON.parse(res.text);
        text.success.should.equal(true);
        text.id.should.equal(simId1);
        text.username.should.equal(user2.username);
        text.read_only.should.equal(true);
        done();
      });
    });
  });
});
