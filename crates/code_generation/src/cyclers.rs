use std::{collections::BTreeSet, iter::once};

use convert_case::{Case, Casing};
use itertools::Itertools;
use proc_macro2::{Ident, TokenStream};
use quote::{format_ident, quote};
use source_analyzer::{
    contexts::Field,
    cyclers::{Cycler, CyclerKind, Cyclers},
    node::Node,
    path::Path,
};
use syn::{
    AngleBracketedGenericArguments, GenericArgument, Path as SynPath, PathArguments, Type, TypePath,
};

use crate::{
    accessor::{path_to_accessor_token_stream, ReferenceKind},
    Execution,
};

pub fn generate_cyclers(cyclers: &Cyclers, mode: Execution) -> TokenStream {
    let recording_frame = if mode == Execution::Run {
        let recording_frame_variants = cyclers.instances().map(|(_cycler, instance)| {
            let instance_name = format_ident!("{}", instance);
            quote! {
                #instance_name {
                    timestamp: std::time::SystemTime,
                    data: std::vec::Vec<u8>,
                },
            }
        });
        quote! {
            pub enum RecordingFrame {
                #(#recording_frame_variants)*
            }
        }
    } else {
        Default::default()
    };
    let cyclers: Vec<_> = cyclers
        .cyclers
        .iter()
        .map(|cycler| generate_module(cycler, cyclers, mode))
        .collect();

    quote! {
        #recording_frame

        #(#cyclers)*
    }
}

fn generate_module(cycler: &Cycler, cyclers: &Cyclers, mode: Execution) -> TokenStream {
    let module_name = format_ident!("{}", cycler.name.to_case(Case::Snake));
    let cycler_instance = generate_cycler_instance(cycler);
    let database_struct = generate_database_struct();
    let cycler_struct = generate_struct(cycler, cyclers, mode);
    let cycler_implementation = generate_implementation(cycler, cyclers, mode);

    quote! {
        #[allow(dead_code, unused_mut, unused_variables, clippy::too_many_arguments, clippy::needless_question_mark, clippy::borrow_deref_ref)]
        pub(crate) mod #module_name {
            use color_eyre::eyre::WrapErr;
            use crate::structs::#module_name::{MainOutputs, AdditionalOutputs};

            #cycler_instance
            #database_struct
            #cycler_struct
            #cycler_implementation
        }
    }
}

fn generate_cycler_instance(cycler: &Cycler) -> TokenStream {
    let instances = cycler
        .instances
        .iter()
        .map(|instance| format_ident!("{}", instance));
    quote! {
        #[derive(Clone, Copy, Debug)]
        pub(crate) enum CyclerInstance {
            #(#instances,)*
        }
    }
}

fn generate_database_struct() -> TokenStream {
    quote! {
        #[derive(Default, serde::Deserialize, serde::Serialize, serialize_hierarchy::SerializeHierarchy)]
        pub(crate) struct Database {
            pub main_outputs: MainOutputs,
            pub additional_outputs: AdditionalOutputs,
        }
    }
}

fn generate_struct(cycler: &Cycler, cyclers: &Cyclers, mode: Execution) -> TokenStream {
    let module_name = format_ident!("{}", cycler.name.to_case(Case::Snake));
    let input_output_fields = generate_input_output_fields(cycler, cyclers);
    let realtime_inputs = match cycler.kind {
        CyclerKind::Perception => quote! {},
        CyclerKind::RealTime => {
            quote! {
                historic_databases: framework::HistoricDatabases<MainOutputs>,
                perception_databases: framework::PerceptionDatabases<crate::perception_databases::Databases>,
            }
        }
    };
    let node_fields = generate_node_fields(cycler);
    let recording_fields = if mode == Execution::Run {
        quote! {
            recording_sender: std::sync::mpsc::SyncSender<crate::cyclers::RecordingFrame>,
            enable_recording: bool,
        }
    } else {
        Default::default()
    };

    quote! {
        pub(crate) struct Cycler<HardwareInterface>  {
            instance: CyclerInstance,
            hardware_interface: std::sync::Arc<HardwareInterface>,
            own_writer: framework::Writer<Database>,
            own_changed: std::sync::Arc<tokio::sync::Notify>,
            own_subscribed_outputs_reader: framework::Reader<std::collections::HashSet<String>>,
            parameters_reader: framework::Reader<crate::structs::Parameters>,
            cycler_state: crate::structs::#module_name::CyclerState,
            #realtime_inputs
            #input_output_fields
            #node_fields
            #recording_fields
        }
    }
}

fn generate_input_output_fields(cycler: &Cycler, cyclers: &Cyclers) -> TokenStream {
    match cycler.kind {
        CyclerKind::Perception => {
            let readers = generate_reader_fields(cyclers);
            quote! {
                own_producer: framework::Producer<MainOutputs>,
                #readers
            }
        }
        CyclerKind::RealTime => {
            let consumers = generate_consumer_fields(cyclers);
            quote! {
                #consumers
            }
        }
    }
}

fn generate_reader_fields(cyclers: &Cyclers) -> TokenStream {
    cyclers
        .instances_with(CyclerKind::RealTime)
        .map(|(cycler, instance)| {
            let field_name = format_ident!("{}_reader", instance.to_case(Case::Snake));
            let cycler_module_name = format_ident!("{}", cycler.name.to_case(Case::Snake));

            quote! {
                #field_name: framework::Reader<crate::cyclers::#cycler_module_name::Database>,
            }
        })
        .collect()
}

fn generate_consumer_fields(cyclers: &Cyclers) -> TokenStream {
    cyclers
        .instances_with(CyclerKind::Perception)
        .map(|(cycler, instance)| {
            let field_name = format_ident!("{}_consumer", instance.to_case(Case::Snake));
            let cycler_module_name = format_ident!("{}", cycler.name.to_case(Case::Snake));

            quote! {
                #field_name: framework::Consumer<crate::structs::#cycler_module_name::MainOutputs>,
            }
        })
        .collect()
}

fn generate_node_fields(cycler: &Cycler) -> TokenStream {
    let fields: Vec<_> = cycler
        .iter_nodes()
        .map(|node| {
            let node_name_snake_case = format_ident!("{}", node.name.to_case(Case::Snake));
            let node_module = &node.module;
            let node_name = format_ident!("{}", node.name);
            quote! {
                #node_name_snake_case: #node_module::#node_name
            }
        })
        .collect();
    quote! {
        #(#fields,)*
    }
}

fn generate_implementation(cycler: &Cycler, cyclers: &Cyclers, mode: Execution) -> TokenStream {
    let new_method = generate_new_method(cycler, cyclers, mode);
    let start_method = match mode {
        Execution::None | Execution::Run => generate_start_method(),
        Execution::Replay => Default::default(),
    };
    let cycle_method = generate_cycle_method(cycler, cyclers, mode);

    quote! {
        impl<HardwareInterface> Cycler<HardwareInterface>
        where
            HardwareInterface: crate::HardwareInterface + Send + Sync + 'static
        {
            #new_method
            #start_method
            #cycle_method
        }
    }
}

fn generate_new_method(cycler: &Cycler, cyclers: &Cyclers, mode: Execution) -> TokenStream {
    let input_output_fields = generate_input_output_fields(cycler, cyclers);
    let cycler_module_name = format_ident!("{}", cycler.name.to_case(Case::Snake));
    let node_initializers = generate_node_initializers(cycler);
    let node_identifiers = cycler
        .iter_nodes()
        .map(|node| format_ident!("{}", node.name.to_case(Case::Snake)));
    let input_output_identifiers = generate_input_output_identifiers(cycler, cyclers);
    let recording_parameter_fields = if mode == Execution::Run {
        quote! {
            recording_sender: std::sync::mpsc::SyncSender<crate::cyclers::RecordingFrame>,
            enable_recording: bool,
        }
    } else {
        Default::default()
    };
    let recording_initializer_fields = if mode == Execution::Run {
        quote! {
            recording_sender,
            enable_recording,
        }
    } else {
        Default::default()
    };

    quote! {
        pub(crate) fn new(
            instance: CyclerInstance,
            hardware_interface: std::sync::Arc<HardwareInterface>,
            own_writer: framework::Writer<Database>,
            own_changed: std::sync::Arc<tokio::sync::Notify>,
            own_subscribed_outputs_reader: framework::Reader<std::collections::HashSet<String>>,
            parameters_reader: framework::Reader<crate::structs::Parameters>,
            #input_output_fields
            #recording_parameter_fields
        ) -> color_eyre::Result<Self> {
            let parameters = parameters_reader.next().clone();
            let mut cycler_state = crate::structs::#cycler_module_name::CyclerState::default();
            #node_initializers
            Ok(Self {
                instance,
                hardware_interface,
                own_writer,
                own_changed,
                own_subscribed_outputs_reader,
                parameters_reader,
                cycler_state,
                #input_output_identifiers
                #(#node_identifiers,)*
                #recording_initializer_fields
            })
        }
    }
}

fn generate_node_initializers(cycler: &Cycler) -> TokenStream {
    let initializers = cycler.iter_nodes().map(|node| {
        let node_name_snake_case = format_ident!("{}", node.name.to_case(Case::Snake));
        let node_module = &node.module;
        let node_name = format_ident!("{}", node.name);
        let field_initializers = generate_node_field_initializers(node, cycler);
        let error_message = format!("failed to create node `{}`", node.name);
        quote! {
            let #node_name_snake_case = #node_module::#node_name::new(
                #node_module::CreationContext::new(
                    #field_initializers
                )
            )
            .wrap_err(#error_message)?;
        }
    });
    quote! {
        #(#initializers)*
    }
}

fn generate_node_field_initializers(node: &Node, cycler: &Cycler) -> TokenStream {
    node.contexts
        .creation_context
        .iter()
        .map(|field| match field {
            Field::AdditionalOutput { name, .. } => {
                panic!("unexpected additional output field `{name}` in CreationContext")
            }
            Field::CyclerState { path, .. } => {
                let accessor = path_to_accessor_token_stream(
                    quote! { cycler_state },
                    path,
                    ReferenceKind::Mutable,
                    cycler,
                );
                quote! {
                    #accessor,
                }
            }
            Field::HardwareInterface { .. } => quote! {
                &hardware_interface,
            },
            Field::HistoricInput { name, .. } => {
                panic!("unexpected historic input field `{name}` in new context")
            }
            Field::Input { name, .. } => {
                panic!("unexpected optional input field `{name}` in new context")
            }
            Field::MainOutput { name, .. } => {
                panic!("unexpected main output field `{name}` in new context")
            }
            Field::Parameter { path, .. } => {
                let accessor = path_to_accessor_token_stream(
                    quote! { parameters },
                    path,
                    ReferenceKind::Immutable,
                    cycler,
                );
                quote! {
                    #accessor,
                }
            }
            Field::PerceptionInput { name, .. } => {
                panic!("unexpected perception input field `{name}` in new context")
            }
            Field::RequiredInput { name, .. } => {
                panic!("unexpected required input field `{name}` in new context")
            }
        })
        .collect()
}

fn generate_input_output_identifiers(cycler: &Cycler, cyclers: &Cyclers) -> TokenStream {
    match cycler.kind {
        CyclerKind::Perception => {
            let readers = generate_reader_identifiers(cyclers);
            quote! {
                own_producer,
                #(#readers,)*
            }
        }
        CyclerKind::RealTime => {
            let consumers = generate_consumer_identifiers(cyclers);
            quote! {
                historic_databases: Default::default(),
                perception_databases: Default::default(),
                #(#consumers,)*
            }
        }
    }
}

fn generate_reader_identifiers(cyclers: &Cyclers) -> Vec<Ident> {
    cyclers
        .instances_with(CyclerKind::RealTime)
        .map(|(_cycler, instance)| format_ident!("{}_reader", instance.to_case(Case::Snake)))
        .collect()
}

fn generate_consumer_identifiers(cyclers: &Cyclers) -> Vec<Ident> {
    cyclers
        .instances_with(CyclerKind::Perception)
        .map(|(_cycler, instance)| format_ident!("{}_consumer", instance.to_case(Case::Snake)))
        .collect()
}

fn generate_start_method() -> TokenStream {
    quote! {
        pub(crate) fn start(
            mut self,
            keep_running: tokio_util::sync::CancellationToken,
        ) -> color_eyre::Result<std::thread::JoinHandle<color_eyre::Result<()>>> {
            let instance_name = format!("{:?}", self.instance);
            std::thread::Builder::new()
                .name(instance_name.clone())
                .spawn(move || {
                    while !keep_running.is_cancelled() {
                        if let Err(error) = self.cycle() {
                            keep_running.cancel();
                            return Err(error).wrap_err_with(|| {
                                format!("failed to execute cycle of cycler `{:?}`", self.instance)
                            });
                        }
                    }
                    Ok(())
                })
                .wrap_err_with(|| {
                    format!("failed to spawn thread for `{instance_name}`")
                })
        }
    }
}

fn generate_cycle_method(cycler: &Cycler, cyclers: &Cyclers, mode: Execution) -> TokenStream {
    let cycle_function_signature = match mode {
        Execution::None | Execution::Run => quote! {
            pub(crate) fn cycle(&mut self) -> color_eyre::Result<()>
        },
        Execution::Replay => quote! {
            pub fn cycle(&mut self, now: std::time::SystemTime, mut recording_frame: &[u8]) -> color_eyre::Result<()>
        },
    };
    let setup_node_executions = cycler
        .setup_nodes
        .iter()
        .map(|node| generate_node_execution(node, cycler, NodeType::Setup, mode));
    let cycle_node_executions = cycler
        .cycle_nodes
        .iter()
        .map(|node| generate_node_execution(node, cycler, NodeType::Cycle, mode));
    let cross_inputs = get_cross_inputs(cycler);
    let cross_inputs = match mode {
        Execution::None => Default::default(),
        Execution::Run => generate_cross_inputs_recording(cycler, cross_inputs),
        Execution::Replay => generate_cross_inputs_extraction(cross_inputs),
    };

    let pre_setup = match mode {
        Execution::None => Default::default(),
        Execution::Run => quote! {
            let enable_recording = self.enable_recording && self.hardware_interface.should_record();
            let mut recording_frame = Vec::new(); // TODO: possible optimization: cache capacity
        },
        Execution::Replay => Default::default(),
    };
    let post_setup = match mode {
        Execution::None => Default::default(),
        Execution::Run => quote! {
            let now = <HardwareInterface as hardware::TimeInterface>::get_now(&*self.hardware_interface);
        },
        Execution::Replay => Default::default(),
    };
    let post_setup = match cycler.kind {
        CyclerKind::Perception => quote! {
            #post_setup
            self.own_producer.announce();
        },
        CyclerKind::RealTime => {
            let perception_cycler_updates = generate_perception_cycler_updates(cyclers);

            quote! {
                #post_setup
                self.perception_databases.update(now, crate::perception_databases::Updates {
                    #perception_cycler_updates
                });
            }
        }
    };
    let lock_readers = match cycler.kind {
        CyclerKind::Perception => cyclers
            .instances_with(CyclerKind::RealTime)
            .map(|(_cycler, instance)| {
                let reader = format_ident!("{}_reader", instance.to_case(Case::Snake));
                let database = format_ident!("{}_database", instance.to_case(Case::Snake));
                quote! {
                    let #database = self.#reader.next();
                }
            })
            .collect(),
        CyclerKind::RealTime => quote! {},
    };
    let after_remaining_nodes = match cycler.kind {
        CyclerKind::Perception => quote! {
            self.own_producer.finalize(own_database_reference.main_outputs.clone());
        },
        CyclerKind::RealTime => quote! {
            self.historic_databases.update(
                now,
                self.perception_databases
                    .get_first_timestamp_of_temporary_databases(),
                &own_database_reference.main_outputs,
            );
        },
    };
    let after_remaining_nodes = match mode {
        Execution::None => after_remaining_nodes,
        Execution::Run => {
            let recording_variants = cycler.instances.iter().map(|instance| {
                let instance_name = format_ident!("{}", instance);
                quote! {
                    CyclerInstance::#instance_name => crate::cyclers::RecordingFrame::#instance_name {
                        timestamp: now,
                        data: recording_frame,
                    },
                }
            });

            quote! {
                #after_remaining_nodes

                if enable_recording {
                    self.recording_sender.try_send(match instance {
                        #(#recording_variants)*
                    }).wrap_err("failed to send recording frame")?;
                }
            }
        }
        Execution::Replay => after_remaining_nodes,
    };

    quote! {
        #[allow(clippy::nonminimal_bool)]
        #cycle_function_signature {
            {
                let instance = self.instance;
                let instance_name = format!("{instance:?}");
                let itt_domain = ittapi::Domain::new(&instance_name);

                let mut own_database = self.own_writer.next();
                let own_database_reference = {
                    use std::ops::DerefMut;
                    own_database.deref_mut()
                };

                #pre_setup

                {
                    let own_subscribed_outputs = self.own_subscribed_outputs_reader.next();
                    let parameters = self.parameters_reader.next();
                    #(#setup_node_executions)*
                }

                #post_setup

                {
                    let own_subscribed_outputs = self.own_subscribed_outputs_reader.next();
                    let parameters = self.parameters_reader.next();
                    #lock_readers
                    #cross_inputs
                    #(#cycle_node_executions)*
                }

                #after_remaining_nodes
            }
            self.own_changed.notify_one();
            Ok(())
        }
    }
}

fn get_cross_inputs(cycler: &Cycler) -> BTreeSet<Field> {
    cycler
        .setup_nodes
        .iter()
        .chain(cycler.cycle_nodes.iter())
        .flat_map(|node| {
            node.contexts
                .cycle_context
                .iter()
                .filter(|field| {
                    matches!(
                        field,
                        Field::CyclerState { .. }
                            | Field::HistoricInput { .. }
                            | Field::Input {
                                cycler_instance: Some(_),
                                ..
                            }
                            | Field::PerceptionInput { .. }
                            | Field::RequiredInput {
                                cycler_instance: Some(_),
                                ..
                            }
                    )
                })
                .cloned()
        })
        .collect()
}

fn generate_cross_inputs_recording(
    cycler: &Cycler,
    cross_inputs: impl IntoIterator<Item = Field>,
) -> TokenStream {
    let recordings = cross_inputs.into_iter().map(|field| {
        let error_message = match &field {
            Field::CyclerState { name, .. } => format!("failed to record cycler state {name}"),
            Field::HistoricInput { name, .. } => format!("failed to record historic input {name}"),
            Field::Input { cycler_instance: Some(_), name, .. } => format!("failed to record input {name}"),
            Field::PerceptionInput { name, .. } => format!("failed to record perception input {name}"),
            Field::RequiredInput { cycler_instance: Some(_), name, .. } => format!("failed to record required input {name}"),
            _ => panic!("unexpected field {field:?}"),
        };
        let value_to_be_recorded = match field {
            Field::CyclerState { path, .. } => {
                let accessor = path_to_accessor_token_stream(
                    quote! { self.cycler_state },
                    &path,
                    ReferenceKind::Immutable,
                    cycler,
                );
                quote! {
                    #accessor
                }
            }
            Field::HistoricInput { path, .. } => {
                let now_accessor = path_to_accessor_token_stream(
                    quote!{ own_database_reference.main_outputs },
                    &path,
                    ReferenceKind::Immutable,
                    cycler,
                );
                let historic_accessor = path_to_accessor_token_stream(
                    quote!{ database },
                    &path,
                    ReferenceKind::Immutable,
                    cycler,
                );
                quote! {
                    &[(now, #now_accessor)]
                        .into_iter()
                        .chain(
                            self
                                .historic_databases
                                .databases
                                .iter()
                                .map(|(system_time, database)| (
                                    *system_time,
                                    #historic_accessor,
                                ))
                        )
                        .collect::<std::collections::BTreeMap<_, _>>()
                }
            }
            Field::Input {
                cycler_instance: Some(cycler_instance),
                path,
                ..
            } => {
                let identifier = format_ident!("{}_database", cycler_instance.to_case(Case::Snake));
                let database_prefix = quote! { #identifier.main_outputs };
                let accessor = path_to_accessor_token_stream(
                    database_prefix,
                    &path,
                    ReferenceKind::Immutable,
                    cycler,
                );
                quote! {
                    &#accessor
                }
            }
            Field::PerceptionInput { cycler_instance, path, .. } => {
                let cycler_instance_identifier =
                    format_ident!("{}", cycler_instance.to_case(Case::Snake));
                let accessor = path_to_accessor_token_stream(
                    quote! { database },
                    &path,
                    ReferenceKind::Immutable,
                    cycler,
                );
                quote! {
                    &[
                        self
                            .perception_databases
                            .persistent()
                            .map(|(system_time, databases)| (
                                *system_time,
                                databases
                                    .#cycler_instance_identifier
                                    .iter()
                                    .map(|database| #accessor)
                                    .collect::<Vec<_>>()
                                ,
                            ))
                            .collect::<std::collections::BTreeMap<_, _>>(),
                        self
                            .perception_databases
                            .temporary()
                            .map(|(system_time, databases)| (
                                *system_time,
                                databases
                                    .#cycler_instance_identifier
                                    .iter()
                                    .map(|database| #accessor)
                                    .collect::<Vec<_>>()
                                ,
                            ))
                            .collect::<std::collections::BTreeMap<_, _>>(),
                    ]
                }
            }
            Field::RequiredInput {
                cycler_instance: Some(cycler_instance),
                path,
                ..
            } => {
                let identifier = format_ident!("{}_database", cycler_instance.to_case(Case::Snake));
                let database_prefix = quote! { #identifier.main_outputs };
                let accessor = path_to_accessor_token_stream(
                    database_prefix,
                    &path,
                    ReferenceKind::Immutable,
                    cycler,
                );
                quote! {
                    &#accessor .unwrap()
                }
            }
            _ => panic!("unexpected field {field:?}"),
        };
        quote! {
            bincode::serialize_into(&mut recording_frame, #value_to_be_recorded).wrap_err(#error_message)?;
        }
    }).collect::<Vec<_>>();

    if recordings.is_empty() {
        return Default::default();
    }

    quote! {
        if enable_recording {
            #(#recordings)*
        }
    }
}

fn generate_cross_inputs_extraction(cross_inputs: impl IntoIterator<Item = Field>) -> TokenStream {
    let extractions = cross_inputs.into_iter().map(|field| {
        let error_message = match &field {
            Field::CyclerState { name, .. } => format!("failed to record cycler state {name}"),
            Field::HistoricInput { name, .. } => format!("failed to record historic input {name}"),
            Field::Input { cycler_instance: Some(_), name, .. } => format!("failed to record input {name}"),
            Field::PerceptionInput { name, .. } => format!("failed to record perception input {name}"),
            Field::RequiredInput { cycler_instance: Some(_), name, .. } => format!("failed to record required input {name}"),
            _ => panic!("unexpected field {field:?}"),
        };
        match field {
            Field::CyclerState { path, .. } => {
                let name = path_to_extraction_variable_name("own", &path, "cycler_state");
                quote! {
                    #[allow(non_snake_case)]
                    let mut #name = bincode::deserialize_from(&mut recording_frame).wrap_err(#error_message)?;
                }
            }
            Field::HistoricInput { path, data_type, .. } => {
                let name = path_to_extraction_variable_name("own", &path, "historic_input");
                quote! {
                    #[allow(non_snake_case)]
                    let #name: std::collections::BTreeMap<std::time::SystemTime, #data_type> = bincode::deserialize_from(&mut recording_frame).wrap_err(#error_message)?;
                }
            }
            Field::Input {
                cycler_instance: Some(cycler_instance),
                path,
                data_type,
                ..
            } => {
                let name = path_to_extraction_variable_name(&cycler_instance, &path, "input");
                quote! {
                    #[allow(non_snake_case)]
                    let #name: #data_type = bincode::deserialize_from(&mut recording_frame).wrap_err(#error_message)?;
                }
            }
            Field::PerceptionInput { cycler_instance, path, data_type, .. } => {
                let name = path_to_extraction_variable_name(&cycler_instance, &path, "perception_input");
                quote! {
                    #[allow(non_snake_case)]
                    let #name: [std::collections::BTreeMap<std::time::SystemTime, Vec<#data_type>>; 2] = bincode::deserialize_from(&mut recording_frame).wrap_err(#error_message)?;
                }
            }
            Field::RequiredInput {
                cycler_instance: Some(cycler_instance),
                path,
                data_type: Type::Path(TypePath {
                    path: SynPath { segments, .. },
                    ..
                }),
                ..
            } if !segments.is_empty() && segments.last().unwrap().ident == "Option" => {
                let name = path_to_extraction_variable_name(&cycler_instance, &path, "required_input");
                let data_type = match &segments.last().unwrap().arguments {
                    PathArguments::AngleBracketed(AngleBracketedGenericArguments {
                        args, ..
                    }) if args.len() == 1 => match args.first().unwrap() {
                        GenericArgument::Type(nested_data_type) => nested_data_type,
                        _ => panic!("unexpected generic argument, expected type argument in data type"),
                    },
                    _ => panic!("expected exactly one generic type argument in data type"),
                };
                quote! {
                    #[allow(non_snake_case)]
                    let #name: #data_type = bincode::deserialize_from(&mut recording_frame).wrap_err(#error_message)?;
                }
            }
            _ => panic!("unexpected field {field:?}"),
        }
    }).collect::<Vec<_>>();

    if extractions.is_empty() {
        return Default::default();
    }

    quote! {
        #(#extractions)*
    }
}

fn path_to_extraction_variable_name(cycler_instance: &str, path: &Path, suffix: &str) -> Ident {
    format_ident!(
        "replay_extraction_{}_{}_{}",
        cycler_instance,
        path.to_segments().join("_"),
        suffix,
    )
}

fn generate_perception_cycler_updates(cyclers: &Cyclers) -> TokenStream {
    cyclers
        .instances_with(CyclerKind::Perception)
        .map(|(_cycler, instance)| {
            let identifier = format_ident!("{}", instance.to_case(Case::Snake));
            let consumer = format_ident!("{}_consumer", identifier);
            quote! {
                #identifier: self.#consumer.consume(now),
            }
        })
        .collect()
}

fn generate_node_execution(
    node: &Node,
    cycler: &Cycler,
    node_type: NodeType,
    mode: Execution,
) -> TokenStream {
    match (node_type, mode) {
        (NodeType::Setup, Execution::Run) => {
            let write_main_outputs_from_node =
                generate_write_main_outputs_from_node(node, cycler, mode);
            let record_main_outputs = generate_record_main_outputs(node);
            quote! {
                #write_main_outputs_from_node
                #record_main_outputs
            }
        }
        (NodeType::Cycle, Execution::Run) => {
            let record_node_state = generate_record_node_state(node);
            let write_main_outputs_from_node =
                generate_write_main_outputs_from_node(node, cycler, mode);
            quote! {
                #record_node_state
                #write_main_outputs_from_node
            }
        }
        (NodeType::Setup, Execution::Replay) => {
            let write_main_outputs_from_frame = generate_write_main_outputs_from_frame(node);
            quote! {
                #write_main_outputs_from_frame
            }
        }
        (NodeType::Cycle, Execution::Replay) => {
            let restore_node_state = generate_restore_node_state(node);
            let write_main_outputs_from_node =
                generate_write_main_outputs_from_node(node, cycler, mode);
            quote! {
                #restore_node_state
                #write_main_outputs_from_node
            }
        }
        (_, Execution::None) => panic!("unexpected Execution::Mode"),
    }
}

fn generate_write_main_outputs_from_node(
    node: &Node,
    cycler: &Cycler,
    mode: Execution,
) -> TokenStream {
    let are_required_inputs_some = generate_required_input_condition(node, cycler);
    let node_name = &node.name;
    let node_member = format_ident!("{}", node.name.to_case(Case::Snake));
    let node_module = &node.module;
    let context_initializers = generate_context_initializers(node, cycler, mode);
    let cycle_error_message = format!("failed to execute cycle of `{}`", node.name);
    let database_updates = generate_database_updates(node);
    let database_updates_from_defaults = generate_database_updates_from_defaults(node);

    quote! {
        {
            #[allow(clippy::needless_else)]
            if #are_required_inputs_some {
                let main_outputs = {
                    let _task = ittapi::Task::begin(&itt_domain, #node_name);
                    self.#node_member.cycle(
                        #node_module::CycleContext::new(
                            #context_initializers
                        ),
                    )
                    .wrap_err(#cycle_error_message)?
                };
                #database_updates
            }
            else {
                #database_updates_from_defaults
            }
        }
    }
}

fn generate_record_main_outputs(node: &Node) -> TokenStream {
    node.contexts
        .main_outputs
        .iter()
        .filter_map(|field| match field {
            Field::MainOutput { name, .. } => {
                let error_message = format!("failed to record {name}");
                Some(quote! {
                    if enable_recording {
                        bincode::serialize_into(&mut recording_frame, &own_database_reference.main_outputs.#name).wrap_err(#error_message)?;
                    }
                })
            },
            _ => None,
        })
        .collect()
}

fn generate_record_node_state(node: &Node) -> TokenStream {
    let node_member = format_ident!("{}", node.name.to_case(Case::Snake));
    let error_message = format!("failed to record `{}`", node.name);
    quote! {
        if enable_recording {
            bincode::serialize_into(&mut recording_frame, &self.#node_member).wrap_err(#error_message)?;
        }
    }
}

fn generate_write_main_outputs_from_frame(node: &Node) -> TokenStream {
    node.contexts
        .main_outputs
        .iter()
        .filter_map(|field| match field {
            Field::MainOutput { name, .. } => {
                let error_message = format!("failed to extract {name}");
                Some(quote! {
                    own_database_reference.main_outputs.#name = bincode::deserialize_from(&mut recording_frame).wrap_err(#error_message)?;
                })
            }
            _ => None,
        })
        .collect()
}

fn generate_restore_node_state(node: &Node) -> TokenStream {
    let node_member = format_ident!("{}", node.name.to_case(Case::Snake));
    let error_message = format!("failed to extract `{}`", node.name);
    quote! {
        {
            use bincode::Options;
            let mut deserializer = bincode::Deserializer::with_reader(
                &mut recording_frame,
                bincode::options()
                    .with_fixint_encoding()
                    .allow_trailing_bytes(),
            );
            serde::Deserialize::deserialize_in_place(
                &mut deserializer,
                &mut self.#node_member,
            ).wrap_err(#error_message)?;
        }
    }
}

enum NodeType {
    Setup,
    Cycle,
}

fn generate_required_input_condition(node: &Node, cycler: &Cycler) -> TokenStream {
    let conditions = node
        .contexts
        .cycle_context
        .iter()
        .filter_map(|field| match field {
            Field::RequiredInput {
                cycler_instance,
                path,
                ..
            } => {
                let database_prefix = match cycler_instance {
                    Some(cycler_instance) => {
                        let identifier =
                            format_ident!("{}_database", cycler_instance.to_case(Case::Snake));
                        quote! { #identifier.main_outputs }
                    }
                    None => {
                        quote! { own_database_reference.main_outputs }
                    }
                };
                let accessor = path_to_accessor_token_stream(
                    database_prefix,
                    path,
                    ReferenceKind::Immutable,
                    cycler,
                );
                Some(quote! {
                    #accessor .is_some()
                })
            }
            _ => None,
        })
        .chain(once(quote! {true}));
    quote! {
        #(#conditions)&&*
    }
}

fn generate_context_initializers(node: &Node, cycler: &Cycler, mode: Execution) -> TokenStream {
    let initializers = node
            .contexts
            .cycle_context
            .iter()
            .map(|field| match field {
                Field::AdditionalOutput {  path, .. } => {
                    let accessor = path_to_accessor_token_stream(
                        quote!{ own_database_reference.additional_outputs },
                        path,
                        ReferenceKind::Mutable,
                        cycler,
                    );
                    let path_string = once("additional_outputs").chain(
                            path.segments.iter().map(|segment| segment.name.as_str())
                        ).join(".");
                    quote! {
                        framework::AdditionalOutput::new(
                            own_subscribed_outputs
                                .iter()
                                .any(|subscribed_output| framework::should_be_filled(subscribed_output, #path_string)),
                            #accessor,
                        )
                    }
                }
                Field::CyclerState { path, .. } => {
                    match mode {
                        Execution::None => Default::default(),
                        Execution::Run => {
                            let accessor = path_to_accessor_token_stream(
                                quote! { self.cycler_state },
                                path,
                                ReferenceKind::Mutable,
                                cycler,
                            );
                            quote! {
                                #accessor
                            }
                        },
                        Execution::Replay => {
                            let name = path_to_extraction_variable_name("own", &path, "cycler_state");
                            quote! {
                                &mut #name
                            }
                        },
                    }
                }
                Field::HardwareInterface { .. } => quote! {
                    &self.hardware_interface
                },
                Field::HistoricInput { path, data_type, .. } => {
                    match mode {
                        Execution::None => Default::default(),
                        Execution::Run => {
                            let now_accessor = path_to_accessor_token_stream(
                                quote!{ own_database_reference.main_outputs },
                                path,
                                ReferenceKind::Immutable,
                                cycler,
                            );
                            let historic_accessor = path_to_accessor_token_stream(
                                quote!{ database },
                                path,
                                ReferenceKind::Immutable,
                                cycler,
                            );
                            quote! {
                                [(now, #now_accessor)]
                                    .into_iter()
                                    .chain(
                                        self
                                            .historic_databases
                                            .databases
                                            .iter()
                                            .map(|(system_time, database)| (
                                                *system_time,
                                                #historic_accessor,
                                            ))
                                    )
                                    .collect::<std::collections::BTreeMap<_, _>>()
                                    .into()
                            }
                        },
                        Execution::Replay => {
                            let name = path_to_extraction_variable_name("own", &path, "historic_input");
                            let is_option = match data_type {
                                Type::Path(TypePath {
                                    path: SynPath { segments, .. },
                                    ..
                                }) => !segments.is_empty() && segments.last().unwrap().ident == "Option",
                                _ => false,
                            };
                            if is_option {
                                quote! {
                                    #name.iter().map(|(key, option_value)| (*key, option_value.as_ref())).collect::<std::collections::BTreeMap<_, _>>().into()
                                }
                            } else {
                                quote! {
                                    #name.iter().map(|(key, option_value)| (*key, option_value)).collect::<std::collections::BTreeMap<_, _>>().into()
                                }
                            }
                        },
                    }
                }
                Field::Input {
                    cycler_instance,
                    path,
                    data_type,
                    ..
                } => {
                    match cycler_instance {
                        Some(cycler_instance) => {
                            match mode {
                                Execution::None => Default::default(),
                                Execution::Run => {
                                    let identifier =
                                        format_ident!("{}_database", cycler_instance.to_case(Case::Snake));
                                    let database_prefix = quote! { #identifier.main_outputs };
                                    let accessor = path_to_accessor_token_stream(
                                        database_prefix,
                                        path,
                                        ReferenceKind::Immutable,
                                        cycler,
                                    );
                                    quote! {
                                        #accessor
                                    }
                                },
                                Execution::Replay => {
                                    let name = path_to_extraction_variable_name(&cycler_instance, &path, "input");
                                    let is_option = match data_type {
                                        Type::Path(TypePath {
                                            path: SynPath { segments, .. },
                                            ..
                                        }) => !segments.is_empty() && segments.last().unwrap().ident == "Option",
                                        _ => false,
                                    };
                                    if is_option {
                                        quote! {
                                            #name.as_ref()
                                        }
                                    } else {
                                        quote! {
                                            &#name
                                        }
                                    }
                                },
                            }
                        }
                        None => {
                            let database_prefix = quote! { own_database_reference.main_outputs };
                            let accessor = path_to_accessor_token_stream(
                                database_prefix,
                                path,
                                ReferenceKind::Immutable,
                                cycler,
                            );
                            quote! {
                                #accessor
                            }
                        }
                    }
                }
                Field::MainOutput { name, .. } => {
                    panic!("unexpected MainOutput `{name}` in cycle context")
                }
                Field::Parameter { path, .. } => {
                    let accessor = path_to_accessor_token_stream(
                        quote! { parameters },
                        path,
                        ReferenceKind::Immutable,
                        cycler,
                    );
                    quote! {
                        #accessor
                    }
                }
                Field::PerceptionInput {
                    cycler_instance,
                    path,
                    data_type,
                    ..
                } => {
                    match mode {
                        Execution::None => Default::default(),
                        Execution::Run => {
                            let cycler_instance_identifier =
                                format_ident!("{}", cycler_instance.to_case(Case::Snake));
                            let accessor = path_to_accessor_token_stream(
                                quote! { database },
                                path,
                                ReferenceKind::Immutable,
                                cycler,
                            );
                            quote! {
                                framework::PerceptionInput {
                                    persistent: self
                                        .perception_databases
                                        .persistent()
                                        .map(|(system_time, databases)| (
                                            *system_time,
                                            databases
                                                .#cycler_instance_identifier
                                                .iter()
                                                .map(|database| #accessor)
                                                .collect()
                                            ,
                                        ))
                                        .collect(),
                                    temporary: self
                                        .perception_databases
                                        .temporary()
                                        .map(|(system_time, databases)| (
                                            *system_time,
                                            databases
                                                .#cycler_instance_identifier
                                                .iter()
                                                .map(|database| #accessor)
                                                .collect()
                                            ,
                                        ))
                                        .collect(),
                                }
                            }
                        },
                        Execution::Replay => {
                            let name = path_to_extraction_variable_name(&cycler_instance, &path, "perception_input");
                            let is_option = match data_type {
                                Type::Path(TypePath {
                                    path: SynPath { segments, .. },
                                    ..
                                }) => !segments.is_empty() && segments.last().unwrap().ident == "Option",
                                _ => false,
                            };
                            let map_operation = if is_option {
                                quote! {
                                    values.iter().map(|option_value| option_value.as_ref()).collect()
                                }
                            } else {
                                quote! {
                                    values.iter().collect()
                                }
                            };
                            quote! {
                                framework::PerceptionInput {
                                    persistent: #name[0].iter().map(|(system_time, values)| (
                                        *system_time,
                                        #map_operation,
                                    )).collect(),
                                    temporary: #name[1].iter().map(|(system_time, values)| (
                                        *system_time,
                                        #map_operation,
                                    )).collect(),
                                }
                            }
                        },
                    }
                }
                Field::RequiredInput {
                    cycler_instance,
                    path,
                    ..
                } => {
                    match cycler_instance {
                        Some(cycler_instance) => {
                            match mode {
                                Execution::None => Default::default(),
                                Execution::Run => {
                                    let identifier =
                                        format_ident!("{}_database", cycler_instance.to_case(Case::Snake));
                                    let database_prefix = quote! { #identifier.main_outputs };
                                    let accessor = path_to_accessor_token_stream(
                                        database_prefix,
                                        path,
                                        ReferenceKind::Immutable,
                                        cycler,
                                    );
                                    quote! {
                                        #accessor .unwrap()
                                    }
                                },
                                Execution::Replay => {
                                    let name = path_to_extraction_variable_name(&cycler_instance, &path, "required_input");
                                    quote! {
                                        &#name
                                    }
                                },
                            }
                        }
                        None => {
                            let database_prefix = quote! { own_database_reference.main_outputs };
                            let accessor = path_to_accessor_token_stream(
                                database_prefix,
                                path,
                                ReferenceKind::Immutable,
                                cycler,
                            );
                            quote! {
                                #accessor .unwrap()
                            }
                        }
                    }
                }
            });
    quote! {
        #(#initializers,)*
    }
}

fn generate_database_updates(node: &Node) -> TokenStream {
    node.contexts
        .main_outputs
        .iter()
        .filter_map(|field| match field {
            Field::MainOutput { name, .. } => Some(quote! {
                own_database_reference.main_outputs.#name = main_outputs.#name.value;
            }),
            _ => None,
        })
        .collect()
}

fn generate_database_updates_from_defaults(node: &Node) -> TokenStream {
    node.contexts
        .main_outputs
        .iter()
        .filter_map(|field| match field {
            Field::MainOutput { name, .. } => {
                let setter = quote! {
                    own_database_reference.main_outputs.#name = Default::default();
                };
                Some(setter)
            }
            _ => None,
        })
        .collect()
}
