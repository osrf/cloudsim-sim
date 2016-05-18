'use strict'

const redis = require("redis")
const client = redis.createClient()

const listName = 'jsgrant'
console.log('ho')

let count = 0
function processItem(item) {
  console.log('process: [' + count + '] ' + item)
  count ++
}

for (var i=0; i < 200; i++) {

  client.rpush( listName, "{" + i + "}")

}

client.lrange(listName, 0, -1, function (error, items) {
  if (error) throw error
  items.forEach(function (item) {
    processItem(item)
  })
})

client.del('jsgrant')
console.log('done')


