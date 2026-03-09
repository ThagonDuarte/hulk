use std::path::PathBuf;

use color_eyre::{
    Result,
    eyre::{Context, OptionExt},
};
use serde::{Deserialize, Serialize};
use tokio::fs::read_to_string;

use crate::Repository;

#[derive(Serialize, Deserialize)]
pub struct Team {
    pub team_number: u8,
    pub robots: Vec<Robot>,
}

#[derive(Clone, Serialize, Deserialize)]
pub struct Robot {
    pub number: u8,
    pub hostname: String,
    pub id: String,
}

impl Robot {
    pub async fn from_team_toml_and_id(hulk_workspace_path: PathBuf, id: String) -> Result<Robot> {
        let team_toml = hulk_workspace_path.join("parameters/team.toml");

        let content = read_to_string(&team_toml)
            .await
            .wrap_err_with(|| format!("failed to read {}", team_toml.display()))?;

        let team: Team = toml::from_str(&content).wrap_err("failed to parse team.toml")?;

        team.robots
            .iter()
            .find(|robot| robot.id == id)
            .cloned()
            .ok_or_eyre(r#"ID "{id}" not found in team.toml"#)
    }
}

impl Repository {
    pub async fn read_team_configuration(&self) -> Result<Team> {
        let team_toml = self.root.join("etc/parameters/team.toml");

        let content = read_to_string(&team_toml)
            .await
            .wrap_err_with(|| format!("failed to read {}", team_toml.display()))?;

        let team = toml::from_str(&content).wrap_err("failed to parse team.toml")?;
        Ok(team)
    }
}
