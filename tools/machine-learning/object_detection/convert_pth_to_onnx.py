from pathlib import Path

import torch


class ActorCritic(torch.nn.Module):
    def __init__(self, num_act, num_obs, num_privileged_obs):
        super().__init__()
        self.critic = torch.nn.Sequential(
            torch.nn.Linear(num_obs + num_privileged_obs, 256),
            torch.nn.ELU(),
            torch.nn.Linear(256, 256),
            torch.nn.ELU(),
            torch.nn.Linear(256, 128),
            torch.nn.ELU(),
            torch.nn.Linear(128, 1),
        )
        self.actor = torch.nn.Sequential(
            torch.nn.Linear(num_obs, 256),
            torch.nn.ELU(),
            torch.nn.Linear(256, 128),
            torch.nn.ELU(),
            torch.nn.Linear(128, 128),
            torch.nn.ELU(),
            torch.nn.Linear(128, num_act),
        )
        self.logstd = torch.nn.parameter.Parameter(
            torch.full((1, num_act), fill_value=-2.0), requires_grad=True
        )

    def act(self, obs):
        action_mean = self.actor(obs)
        action_std = torch.exp(self.logstd).expand_as(action_mean)
        return torch.distributions.Normal(action_mean, action_std)

    def est_value(self, obs, privileged_obs):
        critic_input = torch.cat((obs, privileged_obs), dim=-1)
        return self.critic(critic_input).squeeze(-1)


input_model_path = Path("./t1_walk.pt")
output_model_path = input_model_path.with_suffix(".onnx")

model_weights_old = torch.load(input_model_path, weights_only=False)
# model = ActorCritic(
#     num_act=12, num_obs=47, num_privileged_obs=14
# ).load_state_dict(model_weights_old)
onnx_program = torch.onnx.export(
    model_weights_old, torch.randn(1, 720), output_model_path
)
