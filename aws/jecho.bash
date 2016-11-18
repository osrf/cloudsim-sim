#!/usr/bin/env bash

# reads a json file using node and prints the query

# $1 the json file  (ex: "./options.json")
# $2 the json query (ex: "optionB")

echo `node -pe "var f = \"$1\"; var query = \"$2\"; var j=require(f); j[query] "`


