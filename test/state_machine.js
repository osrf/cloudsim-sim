'use strict'

const state_machine = require('../server/state_machine')
const log = console.log


const proc = state_machine.createMachine()


let transitions = []

// this transition callback is how the state machine
// connects to the simulation server
proc.on("transition", function (data) {
  transitions.push(data)
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
    log('After')
    done()
  })

})
