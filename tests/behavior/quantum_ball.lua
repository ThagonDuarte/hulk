local inspect = require 'inspect'
print("Hello world from lua!")

function spawn_robot(number)
    table.insert(state.robots, create_robot(number))
end

spawn_robot(1)
spawn_robot(2)
spawn_robot(3)
spawn_robot(4)
spawn_robot(5)
spawn_robot(6)
spawn_robot(7)

local game_end_time = 15000.0

function on_goal()
    print("Goal scored, resetting ball!")
    print("Ball: " .. inspect(state.ball))
    print("Ball was at x: " .. state.ball.position[1] .. " y: " .. state.ball.position[2])
    state.ball = nil
    game_end_time = state.cycle_count + 200
end

function on_cycle()
    if state.ball == nil and state.cycle_count % 1000 == 0 then
        print(inspect(state))
        state.ball = {
            position = { 0.0, 0.0 },
            velocity = { 0.0, 0.0 },
        }
    end

    if state.cycle_count == 100 then
        state.game_controller_state.game_state = "Ready"
    end

    if state.cycle_count == 1600 then
        state.game_controller_state.game_state = "Set"
    end

    if state.cycle_count == 1700 then
        state.game_controller_state.game_state = "Playing"
    end

    if state.cycle_count == 1800 then
        state.ball = {
            position = { 0.6, 0.5 },
            velocity = { 0.0, 0.0 },
        }
    end



    if state.cycle_count == 1900 then
        state.ball = {
            position = { -1.77, -2.0 },
            velocity = { 0.0, 0.0 },
        }
    end

    if state.cycle_count == 2100 then
        state.ball = {
            position = { -1.8, 3.0 },
            velocity = { 0.0, 0.0 },
        }
    end

    if state.cycle_count == 2300 then
        state.ball = {
            position = { 3.31, 0.0 },
            velocity = { 0.0, 0.0 },
        }
    end

    if state.cycle_count == 2600 then
        state.ball = {
            position = { 1.31, -1.0 },
            velocity = { 0.0, 0.0 },
        }
    end

    if state.cycle_count == 2900 then
        state.ball = {
            position = { -2.01, 2.0 },
            velocity = { 0.0, 0.0 },
        }
    end

    if state.cycle_count == 3300 then
        state.ball = {
            position = { 1.38, 3.0 },
            velocity = { 0.0, 0.0 },
        }
    end

    if state.cycle_count == 3800 then
        state.ball = {
            position = { -1.51, -2.0 },
            velocity = { 0.0, 0.0 },
        }
    end

    if state.cycle_count == game_end_time then
        state.finished = true
    end
end
