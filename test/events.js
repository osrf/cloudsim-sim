'use strict';

const should = require('should');
const supertest = require('supertest');
const clearRequire = require('clear-require');
var http = require('http');

let csgrant
let app
let agent

let admin
let adminTokenData
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

// received events
let _events = [] 
// mock events-receiver server 
const eventsServer = http.createServer(function (req, res) {
  if (req.method == 'PUT') {
    let body = [];
    req.on('data', function(chunk) {
      body.push(chunk);
    }).on('end', function() {
      body = Buffer.concat(body).toString();
      const ev = JSON.parse(body)
      _events.push(ev)
      res.writeHead(200, { 'Content-Type': 'application/json' });
      res.end(JSON.stringify(ev))
    })
  }
})


describe('<Unit test Events>', function() {

  before(function(done) {
    // Important: the database has to be cleared early, before
    // the server is launched (otherwise, root resources will be missing)
    csgrant = require('cloudsim-grant')
    csgrant.model.clearDb()
    done()
  })

  before(function(done) {
    // To test events we need to first launch an events-receiver server
    // (like 'portal'), and then pass the server url to cloudsim-sim. 
    const getPort = require('get-port');
    getPort().then(port => {
      eventsServer.listen(port)

      const eventsRoute = 'http://127.0.0.1:' + port
      // it is important to void these set ENV values to avoid impacting 
      // other tests 
      process.env.EVENTS_ROUTE = eventsRoute
      const eventsToken = undefined
      process.env.EVENTS_TOKEN = eventsToken
      
      // Now we can launch cloudsim-sim server
      app = require('../server/cloudsim_sim')
      agent = supertest.agent(app)
      done()
    })
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
      csgrant.token.signToken(bobTokenData, (e, tok)=>{
        console.log('token signed for user "bob"')
        if(e) {
          should.fail('sign error: ' + e)
        }
        bobToken = tok
        done()
      })
    })
  })

  describe('<Check initial events>', function() {
    it('after server starts it should be in READY status', function(done) {
      setTimeout(() => {
        const list = _events
        _events = []
        should.equal(list.length, 1)
        should.equal(list[0].sim_status, 'READY')
        done()
      }, 500)
    })
  })

  describe('<Check statuses with a new simulation>', function() {
    it('there should be no simulations', function(done) {
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
        response.result.data.auto.should.equal(true)

        done()
      })
    })
    it('Check published event/statuses of the launched (and stopped) simulation', function(done) {
      setTimeout(() => {
        should.equal(_events.length, 5)
        should.equal(_events[0].sim_status, 'RUNNING')
        should.equal(_events[1].sim_status, 'FINISHING')
        should.equal(_events[2].sim_status, 'FINISHED')
        should.equal(_events[3].sim_status, 'SENDING LOGS')
        should.equal(_events[4].sim_status, 'READY')
        _events = []
        done()
      }, 100)
    })

  })

  describe('<Check statuses with a "auto=false" simulation>', function() {
    it('should be possible for admin to create a simulation', function(done) {
      agent
      .post('/simulations')
      .set('Acccept', 'application/json')
      .set('authorization', adminToken)
      .send({ 
        cmd: 'ls -l',
        stopCmd: 'ls -la',
        auto: false,
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
        should.not.exist(response.result.data.logCmd)
        response.result.data.auto.should.equal(false)
        done()
      })
    })
    it('Check published event/statuses of the new, not started, simulation', function(done) {
      setTimeout(() => {
        should.equal(_events.length, 0)
        done()
      }, 100)
    })

    it('should be possible for admin to update the simulation and make it run', function(done) {
      agent
      .put('/simulations/' + simId1)
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
        response.success.should.equal(true)
        response.requester.should.equal(admin)
        response.id.should.equal(simId1)
        response.result.data.auto.should.equal(true)
        done()
      })
    })
    it('Check published event/statuses of the simulation that just run', function(done) {
      setTimeout(() => {
        should.equal(_events.length, 4)
        should.equal(_events[0].sim_status, 'RUNNING')
        should.equal(_events[1].sim_status, 'FINISHING')
        should.equal(_events[2].sim_status, 'FINISHED')
        should.equal(_events[3].sim_status, 'READY')
        _events = []
        done()
      }, 100)
    })

  })

  describe('<Check events routes>', function() {
    it('should be possible for admin to post to events', function(done) {
      agent
      .post('/events')
      .set('Acccept', 'application/json')
      .set('authorization', adminToken)
      .send({
        postedEvent: 'true'
      })
      .end(function(err,res) {
        res.status.should.be.equal(200)
        res.redirect.should.equal(false)
        var response = parseResponse(res.text)
        response.success.should.equal(true)
        response.requester.should.equal(admin)

        setTimeout(() => {
          should.equal(_events.length, 1)
          should.exist(_events[0].postedEvent)
          should.equal(_events[0].postedEvent, 'true')
          _events = []
          done()
        }, 100)
      })
    })
    it('should not be possible for user Bob to post to events', function(done) {
      agent
      .post('/events')
      .set('Acccept', 'application/json')
      .set('authorization', bobToken)
      .send({
        bobPostedEvent: 2
      })
      .end(function(err,res) {
        res.status.should.be.equal(401)
        res.redirect.should.equal(false)
        var response = JSON.parse(res.text)
        response.success.should.equal(false)
        should.exist(response.error)
        should.equal(_events.length, 0)
        done()
      })
    })
  })

  // after all tests have run, we need to clean up our mess
  after(function(done) {
    process.env.EVENTS_ROUTE = undefined
    process.env.EVENTS_TOKEN = undefined

    csgrant.model.clearDb()
    eventsServer.close(() => {
      console.log("Local events server closed")
    })
    app.close(function() {
      clearRequire.all()
      done()
    })
  })

})
