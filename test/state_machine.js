'use strict'

const state_machine = require('../server/state_machine')
const log = console.log


const proc = state_machine.createMachine()


let transitions = []

proc.on("transition", function (data) {
  transitions.push(data)
})

proc.on("transition", function (data) {
  //console.log('action:', '"' + data.action + '"',
  //            'state:', '"' + this.state + '"',
  //            'prior:', '"' + this.priorState + '"')

  // action: "nothing.boot" state: "ready" prior: "nothing"
  if (data.action === "nothing.boot") {

  }
  // action: "ready.start" state: "prerun" prior: "ready"
  else if (data.action  === "ready.start") {
    // setup (latency)
  }
  // action: "prerun.run" state: "running" prior: "prerun"
  else if (data.action === "prerun.run") {
    // start the simulator
  }
  // action: "running.stop" state: "postrun" prior: "running"
  else if (data.action === "running.stop") {
    // if necessary, stop the simulator
  }
  // action: "postrun.done" state: "ready" prior: "postrun"
  else if (data.action === "postrun.done") {
    // send the logs to the server
  }
  // action: "postrun.done" state: "ready" prior: "postrun"
  else if(data.action === "postrun.done") {
    // nothing to do (phone home?)
  }
  // oops ... this is not a state we expected
  else {
    throw 'state "' + this.state + '" is not recognized'
  }
})


log(transitions)

const util = require('util');
const should = require('should');
const csgrant = require('cloudsim-grant')
const token = csgrant.token

describe('<Unit test State machine>', function() {

  before(function(done) {
    log('before')
    proc.state.should.equal("nothing")
    done()
  })

  describe('bootstraping the state machine', function() {
    it('it should be done (manually)', function(done) {
      transitions = []
      proc.boot()
      transitions.length.should.equal(1)
      transitions[0].action.should.equal('nothing.boot')
      proc.state.should.equal("ready")
      done()
    })
  })

  describe('can\'t boot twice', function() {
    it('it should make 0 transitions', function(done) {
      transitions = []
      proc.boot()
      transitions.length.should.equal(0)
      proc.state.should.equal('ready')
      done()
    })
  })


  describe('start a simulation', function() {
    it('it should make 2 transitions', function(done) {
      transitions = []
      proc.start()
      transitions.length.should.equal(2)
      // this one to set the latency and other things
      transitions[0].action.should.equal('ready.start')
      // this one to start the simulator process
      transitions[1].action.should.equal('prerun.run')
      proc.state.should.equal('running')
      done()
    })
  })

  describe('can\'t restart', function() {
    it('it should not start when running', function(done) {
      transitions = []
      proc.start()
      transitions.length.should.equal(0)
      proc.state.should.equal('running')
      done()
    })
  })

  describe('stop a simulation', function() {
    it('it should make 2 transitions', function(done) {
      transitions = []
      proc.stop()
      transitions.length.should.equal(2)
      // this one to stop the simulator process
      transitions[0].action.should.equal('running.stop')
      // this one to send the log files
      transitions[1].action.should.equal('postrun.done')
      proc.state.should.equal('ready')
      done()
    })
  })

  describe('can\'t stop', function() {
    it('it should stay ready', function(done) {
      transitions = []
      proc.stop()
      transitions.length.should.equal(0)
      proc.state.should.equal('ready')
      done()
    })
  })

  describe('can start', function() {
    it('it should stay ready', function(done) {
      transitions = []
      proc.start()
      transitions.length.should.equal(2)
      proc.state.should.equal('running')
      done()
    })
  })


  after(function(done) {
    log('After')
    done()
  })

})
