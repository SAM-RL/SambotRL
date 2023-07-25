from gym.envs.registration import register

register(
    id='field-v0',
    entry_point='gym_field.envs:SpatialTemporalFieldEnv'
)

register(
    id='field-with-time-v0',
    entry_point='gym_field.envs:SpatialTemporalFieldWithTimeEnv'
)

register(
    id='field-with-grad-v0',
    entry_point='gym_field.envs:SpatialTemporalFieldWithGradConc'
)

register(
    id='field-with-grad-5-actions-v0',
    entry_point='gym_field.envs:SpatialTemporalFieldWithGradConcFiveActions'
)

register(
    id='field-with-grad-9-actions-v0',
    entry_point='gym_field.envs:SpatialTemporalFieldWithGradConcNineActions'
)

register(
    id='field-with-grad-reward-9-actions-v0',
    entry_point='gym_field.envs:SpatialTemporalFieldWithGradRewardConcNineActions'
)

register(
    id='field-no-loc-state-with-grad-reward-9-actions-v0',
    entry_point='gym_field.envs:SpatialTemporalFieldNoLocStateWithGradRewardConcNineActions'
)
