
-- toggle a relay at 5Hz

local RELAY_NUM = 0

function update() -- this is the loop which periodically runs
  if arming:is_armed() then
    relay:on(RELAY_NUM)
  else
    relay:off(RELAY_NUM)
  end 
  return update, 200 -- reschedules the loop at 5Hz
end

return update() -- run immediately before starting to reschedule
