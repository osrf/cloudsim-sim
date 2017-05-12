'use strict'

const express = require('express')
const app = express()
const fs = require('fs')
const bodyParser = require("body-parser")
const cors = require('cors')
const morgan = require('morgan')
const dotenv = require('dotenv')
const path = require('path')
// cloudsim module
const csgrant = require('cloudsim-grant')
// local modules
const simulations = require('./simulations')
const downloads = require('./downloads')
const events = require('./events')

// the configuration values are set in the local .env file
// this loads the .env content and puts it in the process environment.
dotenv.load()

let httpServer = null

const useHttps = false
if(useHttps) {
  const privateKey  = fs.readFileSync(__dirname + '/key.pem', 'utf8')
  const certificate = fs.readFileSync(__dirname + '/key-cert.pem', 'utf8')
  httpServer = require('https').Server({
    key: privateKey, cert: certificate
  }, app)
}
else {
  httpServer = require('http').Server(app)
}


// The Cross-Origin Resource Sharing standard
app.use(cors())
// Populates the body of each request
app.use(bodyParser.json())

// prints all requests to the terminal
app.use(morgan('combined', {
  skip: function (req) {
    // skip /api stuff
    const isApi = req.originalUrl.startsWith('/api/')
    if (isApi) {
      return true
    }
    return false
  }
}))

// the port of the server
const port = process.env.PORT || 4000
// the delay between simulator process state machine update in ms
let simulationsSchedulerInterval = process.env.SCHEDULER_INTERVAL || 1000
if (process.env.NODE_ENV === 'test') {
  simulationsSchedulerInterval = 1
}

const adminUser = process.env.CLOUDSIM_ADMIN || 'admin'

// setup
// error if file is not there
// TODO: it seems these keys.zip are not actually used afterwards. Can we delete them?
const pathToKeysFile = __dirname + '/keys.zip'
fs.statSync(pathToKeysFile)

const dbName = 'cloudsim-sim' + (process.env.NODE_ENV == 'test'? '-test': '')
const dbUrl = '127.0.0.1'

function details() {
  const date = new Date()
  const version = require('../package.json').version
  const csgrantVersion = require('cloudsim-grant/package.json').version
  const env = app.get('env')
  const s = `
date: ${date}
cloudsim-sim version: ${version}
port: ${port}
cloudsim-grant version: ${csgrantVersion}
admin user: ${adminUser}
environment: ${env}
redis database name: ${dbName}
redis database url: ${dbUrl}
path to keys: ${pathToKeysFile}
`
  return s
}


// write details to the console
console.log('============================================')
console.log(details())
console.log('============================================')

app.use("/api", express.static(path.join(__dirname, '/../api')));

app.get('/', function (req, res) {
  const info = details()
  const s = `
    <h1>Cloudsim-sim server</h1>
    <div>Gazebo controller is running</div>
    <pre>
  	${info}
    </pre>
  `
  res.end(s)
})

// setup the /permissions routes
csgrant.setPermissionsRoutes(app)

// local modules have routes too
simulations.setRoutes(app)
downloads.setRoutes(app)

// shutdown nicely
process.on('SIGTERM', ()=>{
  console.log('SIGTERM')
  simulations.stopSimulationScheduler()
})

// make csgrant available to tests
app.csgrant = csgrant

// Expose app
exports = module.exports = app;
app.close = function(cb) {
  console.log('MANUAL SERVER SHUTDOWN')
  const socketsDict = csgrant.sockets.getUserSockets()
  socketsDict.io.close()
  httpServer.close(cb)
}

// we create 2 initial resources,
// load the database and start serving
// when ready
// these may be overriden by the options.json
let resources = [
  {
    "name": "simulations",
    "data": {},
    "permissions": [
      {
        "username": adminUser,
        "permissions": {
          "readOnly": false
        }
      }
    ]
  },
  {
    "name": "downloads",
    "data": {
      "path": pathToKeysFile
    },
    "permissions": [
      {
        "username": adminUser,
        "permissions": {
          "readOnly": false
        }
      }
    ]
  }
]

console.log("TOKEN", process.env.EVENTS_TOKEN)

let eventsRoute = process.env.EVENTS_ROUTE
let eventsToken = process.env.EVENTS_TOKEN
console.log('loading options.json...')
try {
  const options = require('../options.json')
  if (options.resources) {
    console.log('replacing default options with:' + options.resources)
    resources = options.resources
  }
  eventsRoute = options.simulation_data_route || eventsRoute
  eventsToken = options.token || eventsToken
}
catch(e) {
  console.log('Can\'t load ./options.json: ' + e)
}

events.init(eventsRoute, eventsToken)
csgrant.init(resources,
  dbName,
  dbUrl,
  httpServer,
  (err)=> {
    if(err) {
      console.log('Error loading resources: ' + err)
      process.exit(-2)
    }
    else {
      console.log('resources loaded')
      // Run the simulator process scheduler
      simulations.startSimulationsScheduler(simulationsSchedulerInterval)

      // start the server
      httpServer.listen(port, function(){
        console.log('ssl: ' + useHttps)
        console.log('listening on *:' + port);
      })
      csgrant.dump()
    }
  }
)
