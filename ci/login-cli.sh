# Emulates a CLI login by copying the file
mkdir -p ~/.particle
travis decrypt cli-deets.secret ~/.particle/particle.config.json
