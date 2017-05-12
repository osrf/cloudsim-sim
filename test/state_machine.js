'use strict'

const state_machine = require('../server/state_machine')
const clearRequire = require('clear-require');

// we use this list to record the successive transitions, and
// verify that they are correct in the tests
let transitions = []

const proc = state_machine.createMachine()

// add the necessary callbacks. For testing,
proc.bootStateMachine = function() {
  transitions.push('bootStateMachine')
}

proc.getReadyToRunSimulator = function() {
  transitions.push('getReadyToRunSimulator')
}

proc.startTheSimulator= function() {
  transitions.push('startTheSimulator')
}

proc.stopTheSimulator = function(done) {
  transitions.push('stopTheSimulator')
  done()
}

proc.sendLogs = function(done) {
  transitions.push('sendLogs')
  done()
}

describe('<Unit test State machine>', function() {

  before(function(done) {
    proc.state.should.equal("nothing")
    done()
  })

  describe('bootstraping the state machine', function() {
    it('it should be done (manually)', function(done) {
      transitions = []
      proc.boot()
      transitions.length.should.equal(1)
      transitions[0].should.equal('bootStateMachine')
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
      transitions[0].should.equal('getReadyToRunSimulator')
      // this one to start the simulator process
      transitions[1].should.equal('startTheSimulator')
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
      transitions[0].should.equal('stopTheSimulator')
      // this one to send the log files
      transitions[1].should.equal('sendLogs')
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

  describe('can start again', function() {
    it('it should stay ready', function(done) {
      transitions = []
      proc.start()
      transitions.length.should.equal(2)
      proc.state.should.equal('running')
      done()
    })
  })

  after(function(done) {
    clearRequire.all()
    done()
  })
})
