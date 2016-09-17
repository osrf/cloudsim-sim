'use strict'

const machina = require('machina')
const log = console.log

// The state machine configuration. It defines the states and transitions.
let config = {
  boot: function() {
    this.handle("boot")
  },
  start: function() {
    this.handle("start")
    this.handle("run")
  },
//  run: function() {
//    this.handle("run")
//  },
  stop: function() {
    this.handle("stop")
    this.handle("done")
  },
  initialState: "nothing",
  states: {
    "nothing": {
      "boot": function () {
        this.transition("ready")
      },
    },
    "ready": {
      "start": function () {
        this.transition("prerun")
      },
    },
    "prerun": {
      "run": function () {
        this.transition("running")
      }
    },
    "running": {
      "stop": function () {
        this.transition("postrun")
      },
    },
    "postrun": {
      "done": function () {
        this.transition("ready")
      },
    },
  },
}

// A function that creates a process state machine
exports.createMachine = function () {
  return new machina.Fsm( config )
}


