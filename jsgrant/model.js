'use strict'

const redis = require("redis")
const client = redis.createClient()

// the database list where data is saved
let listName = 'jsgrant'

if (process.env.NODE_ENV === "test") {
  console.log('jsgrant/model.js NODE_ENV: ' + process.env.NODE_ENV)
  // test mode...
  // use the test list instead of the live one
  listName = 'jsgrant_test'
}


// inter
function push(operation, resource, data) {
  const json = JSON.stringify({
    operation: operation,
    resource: resource,
    data: data
  })
  client.rpush( listName, json)
}

function revoke(granter, grantee, resource, readOnly ) {
  const data = {granter: granter,
                grantee: grantee,
                readOnly: readOnly}
  push('revoke', resource, data)
}

// assign data for a new resource
function setResource(owner, resource, data) {
  push('set', resource, data)
}

function grant(granter, grantee, resource, readOnly ) {
  const data = {granter: granter,
                grantee: grantee,
                readOnly: readOnly}
  push('grant', resource, data)
}

// this function expects a callback for each item with the following interface
//  callback (err, items) where items is a list in which each item is an
//  object with the following keys:
//    operation, resource, data
function readDb(cb) {
  // get all data from db
  client.lrange(listName, 0, -1, function (error, items) {
    if (error)
      cb(error)
    // transform items from strings to data
    for (var i =0; i < items.length; i++) {
      items [i] = JSON.parse(items[i])
    }
    // return the data
    cb(null, items)
  })
}

function clearDb() {
  client.del(listName)
}

exports.clearDb = clearDb
exports.grant = grant
exports.revoke = revoke
exports.setResource = setResource
exports.readDb = readDb




