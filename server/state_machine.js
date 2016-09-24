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
  const proc = new machina.Fsm(config)

// This is state machine state changes callback.
// This is where simulators are started and stopped
proc.on("transition", function (data) {
  // action: "nothing.boot" state: "ready" prior: "nothing"
  if (data.action === "nothing.boot") {
    this.bootStateMachine()
    console.log('state machine ready')
  }
  // action: "ready.start" state: "prerun" prior: "ready"
  else if (data.action  === "ready.start") {
    // setup (latency)
    this.getReadyTorunSimulator()
    console.log('Pre simulation run setup')
  }
  // action: "prerun.run" state: "running" prior: "prerun"
  else if (data.action === "prerun.run") {
    // start the simulator
    this.startTheSimulator()
    console.log('simulator started')
  }

  // action: "running.stop" state: "postrun" prior: "running"
  else if (data.action === "running.stop") {
    this.stopTheSimulator()
    console.log('Simulation stopped')
  }
  // action: "postrun.done" state: "ready" prior: "postrun"
  else if (data.action === "postrun.done") {
    // send the logs to the server
    this.sendLogs()
    console.log('logs sent')
  }
  // action: "postrun.done" state: "ready" prior: "postrun"
  else if(data.action === "postrun.done") {
    // clean up
    this.cleanUpAfterRun()
  }
  // oops ... this is not a state we expected
  else {
    throw 'state "' + this.state + '" is not recognized'
  }
})

  return proc
}


