'use strict'

var f = "/home/ubuntu/code/cloudsim-options.json"
var j = require(f)
var key = j['github_deploy_key']

const privEnv = key.replace( new RegExp( "\\n", "g" ),"\n")
console.log(privEnv)
