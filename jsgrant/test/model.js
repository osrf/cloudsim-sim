'use strict'

const should = require('should')

console.log('NODE_ENV: ' + process.env.NODE_ENV)
// this is the db interface
const model = require('../model')


describe('<Unit Test>', function() {

    before(function(done) {
        console.log('erasing data before')
        model.clearDb()
        done()
    })

    describe('adding and sharing a resource:', function(){
      it('should have an empty db', (done) => {
        model.readDb((err, items)=>{
          if(err)
            should.fail(err)
          if(items.length != 0){
            should.fail("database not empty items: " + items.length)
          }
          else {
            done()
          }
        })
      })

      it('should have a toaster', (done) => {
        model.setResource('me', 'toaster', {capacity:2})
        model.readDb((err, items)=>{
          if(err)
            should.fail(err)

          if(items.length != 1) {
            should.fail('no toaster added: (' + items.length + ' items)')
          }
          else {
            if(items[0].data.capacity != 2)
              should.fail('not our toaster')
            done()
          }
        })
      })

      it('should be possible to share the toaster with joe', (done) => {
        model.grant('me', 'joe', 'toaster', true)
        model.readDb((err, items)=>{
          if(err)
            should.fail(err)

          if(items.length != 2) {
            should.fail('not granted')
          }
          else {
            done()
          }
        })
      })

      it('should be possible to revoke the toaster for joe', (done) => {
        model.revoke('me', 'joe', 'toaster', true)
        model.readDb((err, items)=>{
          if(err)
            should.fail(err)

          if(items.length != 3) {
            should.fail('not revoked')
          }
          else {
            done()
          }
        })
      })

      it('should be possible to remove the toaster', (done) => {
        model.setResourse('me', 'toaster', null)
        model.readDb((err, items)=>{
          if(err)
            should.fail(err)

          if(items.length != 3) {
            should.fail('not revoked')
          }
          else {
            done()
          }
        })
      })

    })
})
